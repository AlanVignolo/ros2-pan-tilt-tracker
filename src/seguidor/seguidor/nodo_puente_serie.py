"""Puente entre ROS 2 y el firmware de la NUCLEO por puerto serie."""

import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import serial
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger

PUERTO_POR_DEFECTO = (
    '/dev/serial/by-id/'
    'usb-STMicroelectronics_STM32_STLink_0670FF535251667187024749-if02')


class NodoPuenteSerie(Node):
    """Traduce /cmd_angulos y /homing al protocolo ASCII del micro y publica su estado."""

    def __init__(self):
        super().__init__('nodo_puente_serie')

        # Ruta by-id en lugar de /dev/ttyACMx: no cambia si se reconecta el USB
        # o si aparece otro dispositivo serie.
        self.declare_parameter('puerto', PUERTO_POR_DEFECTO)
        self.declare_parameter('baudios', 115200)
        self.puerto = self.get_parameter('puerto').value
        self.baudios = self.get_parameter('baudios').value

        self.habilitado = None    # None hasta que el micro informe el interruptor

        qos_estado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_habilitado = self.create_publisher(Bool, '/motores_habilitados', qos_estado)

        self.serie = None
        self.abrir_puerto()
        self.create_timer(1.0, self.vigilar_puerto)
        self.create_timer(0.01, self.leer_puerto)

        self.create_subscription(
            Float64MultiArray, '/cmd_angulos', self.recibir_angulos, 10)
        self.create_service(Trigger, '/homing', self.atender_homing)

    def abrir_puerto(self):
        try:
            # Timeout corto para que readline nunca bloquee el executor.
            self.serie = serial.Serial(self.puerto, self.baudios, timeout=0.1)
            self.get_logger().info(f'Puerto {self.puerto} abierto a {self.baudios} baudios')
            # El micro puede haber arrancado antes que este nodo y su aviso EN
            # inicial se perdió. Se le pide el estado de nuevo.
            self.serie.write(b'S\n')
        except (serial.SerialException, OSError) as e:
            self.serie = None
            self.get_logger().warning(
                f'No se pudo abrir el puerto: {e}', throttle_duration_sec=10.0)

    def cerrar_puerto(self):
        if self.serie is not None:
            try:
                self.serie.close()
            except (serial.SerialException, OSError):
                pass
            self.serie = None

    def vigilar_puerto(self):
        """Detecta la desconexión del USB y reintenta abrir el puerto."""
        if self.serie is not None and not os.path.exists(self.puerto):
            self.get_logger().error('El micro se desconectó')
            self.cerrar_puerto()
        if self.serie is None:
            self.abrir_puerto()

    def recibir_angulos(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warning(
                f'Se esperaban 2 valores [pan, tilt], llegaron {len(msg.data)}')
            return

        if self.serie is None:
            self.get_logger().warning(
                'Sin conexión con el micro, orden descartada', throttle_duration_sec=2.0)
            return

        # Décimas de grado como enteros, así el firmware no necesita parsear floats.
        pan = round(msg.data[0] * 10)
        tilt = round(msg.data[1] * 10)
        orden = f'P{pan} T{tilt}\n'

        try:
            self.serie.write(orden.encode('ascii'))
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Se perdió la conexión: {e}')
            self.cerrar_puerto()

    def atender_homing(self, pedido, respuesta):
        """
        Inicia el homing en el micro.

        Responde apenas se envía la orden. El resultado llega más tarde por el
        puerto serie y lo informa procesar_linea.
        """
        if self.serie is None:
            respuesta.success = False
            respuesta.message = 'Sin conexión con el micro'
            return respuesta
        if self.habilitado is False:
            respuesta.success = False
            respuesta.message = 'Motores deshabilitados: encendé el interruptor'
            return respuesta

        try:
            self.serie.write(b'H\n')
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Se perdió la conexión: {e}')
            self.cerrar_puerto()
            respuesta.success = False
            respuesta.message = 'Se perdió la conexión con el micro'
            return respuesta

        respuesta.success = True
        respuesta.message = 'Homing iniciado: el resultado aparece en el log del puente'
        return respuesta

    def leer_puerto(self):
        """Procesa todas las líneas pendientes en el buffer de entrada."""
        if self.serie is None:
            return
        try:
            while self.serie.in_waiting > 0:
                linea = self.serie.readline().decode('ascii', errors='replace').strip()
                if linea:
                    self.procesar_linea(linea)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Se perdió la conexión: {e}')
            self.cerrar_puerto()

    def procesar_linea(self, linea):
        if linea in ('EN0', 'EN1'):
            self.actualizar_habilitado(linea == 'EN1')
        elif linea.startswith('ST'):
            for campo in linea.split():
                if campo in ('EN0', 'EN1'):
                    self.actualizar_habilitado(campo == 'EN1')
        elif linea.startswith('OK'):
            # En debug porque el controlador genera varias por segundo.
            self.get_logger().debug(linea)
        elif linea == 'OFF':
            self.get_logger().warning(
                'Motores deshabilitados: orden descartada', throttle_duration_sec=2.0)
        elif linea == 'HOME START':
            self.get_logger().info('Homing en curso')
        elif linea.startswith('HOME OK'):
            # Formato "HOME OK R711": rango útil en décimas de grado a cada lado del centro.
            rango = int(linea.split()[-1][1:]) / 10.0
            self.get_logger().info(f'Homing completo: rango útil ±{rango:.1f}°')
        elif linea.startswith('HOME ERR'):
            motivo = linea[len('HOME ERR '):]
            self.get_logger().error(f'El homing falló: {motivo}')
        elif linea == 'BUSY':
            self.get_logger().warning(
                'Homing en curso: orden descartada', throttle_duration_sec=2.0)
        else:
            self.get_logger().warning(f'Línea inesperada del micro: "{linea}"')

    def actualizar_habilitado(self, valor):
        if valor == self.habilitado:
            return
        self.habilitado = valor
        self.pub_habilitado.publish(Bool(data=valor))
        if valor:
            self.get_logger().info('Motores habilitados')
        else:
            self.get_logger().warning('Motores deshabilitados')

    def destroy_node(self):
        self.cerrar_puerto()
        super().destroy_node()


def main():
    rclpy.init()
    nodo = NodoPuenteSerie()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
