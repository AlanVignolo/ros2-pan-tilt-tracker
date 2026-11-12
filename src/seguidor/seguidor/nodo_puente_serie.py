import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile # qos: Quality of Service, para publicar el estado del interruptor
from std_msgs.msg import Bool, Float64MultiArray
import serial


class NodoPuenteSerie(Node):
    """Recibe ángulos por ROS2 y se los envía al micro por puerto serie."""

    def __init__(self):
        super().__init__('nodo_puente_serie')

        # Ruta by-id: estable aunque cambie el /dev/ttyACMx asignado
        self.declare_parameter('puerto', '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0670FF535251667187024749-if02')
        self.declare_parameter('baudios', 115200)
        self.puerto = self.get_parameter('puerto').value
        self.baudios = self.get_parameter('baudios').value

        # Estado del interruptor de la Nucleo. None = todavía no se sabe.
        self.habilitado = None
        # Publicador del estado.
        qos_estado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) # TRANSIENT_LOCAL: guarda el último mensaje
        # y se lo entrega a cualquier nodo que se suscriba después.
        self.pub_habilitado = self.create_publisher(Bool, '/motores_habilitados', qos_estado)

        self.serie = None
        self.abrir_puerto()
        self.create_timer(1.0, self.vigilar_puerto)  # Watchdog de conexión, reconecta solo
        self.create_timer(0.01, self.leer_puerto) # Lee el puerto cada 10 ms y procesa todas las líneas que hayan llegado

        # Mensaje esperado: data = [pan, tilt], en grados
        self.create_subscription(
            Float64MultiArray, '/cmd_angulos', self.recibir_angulos, 10)

    def abrir_puerto(self):
        try:
            # timeout acotado: readline nunca bloquea el executor
            self.serie = serial.Serial(self.puerto, self.baudios, timeout=0.1)
            self.get_logger().info(f'Puerto {self.puerto} abierto a {self.baudios} baudios')
            self.serie.write(b'S\n') # Solicita estado del interruptor la conexion por si el micro arranco antes
        except (serial.SerialException, OSError) as e:
            self.serie = None
            self.get_logger().warning(  # throttle: limita el spam mientras reintenta
                f'No se pudo abrir el puerto: {e}', throttle_duration_sec=10.0)

    def cerrar_puerto(self):
        if self.serie is not None:
            try:
                self.serie.close()
            except (serial.SerialException, OSError):
                pass
            self.serie = None

    def vigilar_puerto(self):
        if self.serie is not None and not os.path.exists(self.puerto):  # Detecta desconexión USB
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

        # Décimas de grado como entero: evita parsear floats en el firmware
        pan = round(msg.data[0] * 10)
        tilt = round(msg.data[1] * 10)
        orden = f'P{pan} T{tilt}\n'  # Protocolo ASCII: "P<pan> T<tilt>\n"

        try:
            # Solo se escribe. La respuesta la lee y clasifica leer_puerto.
            self.serie.write(orden.encode('ascii'))
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Se perdió la conexión: {e}')
            self.cerrar_puerto()  # El watchdog se encarga de reconectar

    def leer_puerto(self):
        """Lee todas las líneas que llegaron del micro y las procesa."""
        if self.serie is None:
            return
        try:
            # in_waiting: cuántos bytes llegaron y todavía no se leyeron.
            # Se repite hasta vaciar lo recibido.
            while self.serie.in_waiting > 0:
                linea = self.serie.readline().decode('ascii', errors='replace').strip()
                if linea:
                    self.procesar_linea(linea)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Se perdió la conexión: {e}')
            self.cerrar_puerto()  # El watchdog se encarga de reconectar

    def procesar_linea(self, linea):
        """Clasifica una línea del micro según lo que dice."""
        if linea in ('EN0', 'EN1'):
            # Aviso del interruptor
            self.actualizar_habilitado(linea == 'EN1') # True si habilitado, False si deshabilitado
        elif linea.startswith('ST'):
            # Respuesta a "S": se busca el campo EN entre los demás
            for campo in linea.split():
                if campo in ('EN0', 'EN1'):
                    self.actualizar_habilitado(campo == 'EN1')
        elif linea.startswith('OK'):
            # Orden ejecutada. Nivel debug: con el controlador llegan varias
            # por segundo y en INFO llenarían la terminal.
            self.get_logger().debug(linea)
        elif linea == 'OFF':
            self.get_logger().warning(
                'Motores deshabilitados: orden descartada', throttle_duration_sec=2.0)
        else:
            self.get_logger().warning(f'Línea inesperada del micro: "{linea}"')

    def actualizar_habilitado(self, valor):
        """Publica el estado del interruptor, solo si cambió."""
        if valor == self.habilitado:
            return
        self.habilitado = valor
        msg = Bool()
        msg.data = valor
        self.pub_habilitado.publish(msg)
        if valor:
            self.get_logger().info('Motores habilitados')
        else:
            self.get_logger().warning('Motores deshabilitados')

    def destroy_node(self):
        self.cerrar_puerto()  # Libera el puerto al apagar el nodo
        super().destroy_node()


def main():
    rclpy.init()
    nodo = NodoPuenteSerie()
    try:
        rclpy.spin(nodo)  # Bloquea y despacha callbacks hasta el shutdown
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
