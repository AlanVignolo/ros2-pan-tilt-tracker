import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
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

        self.serie = None
        self.abrir_puerto()
        self.create_timer(1.0, self.vigilar_puerto)  # Watchdog de conexión, reconecta solo

        # Mensaje esperado: data = [pan, tilt], en grados
        self.create_subscription(
            Float64MultiArray, '/cmd_angulos', self.recibir_angulos, 10)

    def abrir_puerto(self):
        try:
            # timeout acotado: readline nunca bloquea el executor
            self.serie = serial.Serial(self.puerto, self.baudios, timeout=0.1)
            self.get_logger().info(f'Puerto {self.puerto} abierto a {self.baudios} baudios')
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
            self.serie.write(orden.encode('ascii'))
            respuesta = self.serie.readline().decode('ascii', errors='replace').strip()  # ACK
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Se perdió la conexión: {e}')
            self.cerrar_puerto()  # El watchdog se encarga de reconectar
            return

        if respuesta.startswith('OK'):
            self.get_logger().info(f'{orden.strip()} -> {respuesta}')
        else:
            self.get_logger().warning(
                f'{orden.strip()} -> respuesta inesperada: "{respuesta}"')

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
