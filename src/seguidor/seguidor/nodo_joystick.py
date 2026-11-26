"""Receptor UDP del joystick inalámbrico (ESP32)."""

import socket
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


class NodoJoystick(Node):
    """Recibe paquetes "J <x> <y> <manual>" por UDP y los publica en /joy."""

    def __init__(self):
        super().__init__('nodo_joystick')

        self.declare_parameter('puerto', 5005)
        self.declare_parameter('timeout', 0.5)    # s sin paquetes para considerar la señal perdida
        puerto = self.get_parameter('puerto').value
        self.timeout = self.get_parameter('timeout').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', puerto))
        # No bloqueante: el socket se lee desde un timer y nunca debe trabar el executor.
        self.sock.setblocking(False)

        self.pub = self.create_publisher(Joy, '/joy', 10)

        self.t_ultimo_paquete = 0.0
        self.con_senal = False

        self.create_timer(0.01, self.leer_socket)
        self.get_logger().info(f'Escuchando el joystick en el puerto UDP {puerto}')

    def leer_socket(self):
        while True:
            try:
                datos, _ = self.sock.recvfrom(64)
            except BlockingIOError:
                break
            self.procesar_paquete(datos.decode('ascii', errors='replace').strip())

        # Si se corta el WiFi los ejes quedarían en su último valor y la cámara
        # seguiría moviéndose. Al perder la señal se publica todo en cero una vez.
        if self.con_senal and time.monotonic() - self.t_ultimo_paquete > self.timeout:
            self.con_senal = False
            self.get_logger().warning('Sin señal del joystick: ejes en cero')
            self.publicar(0.0, 0.0, 0)

    def procesar_paquete(self, texto):
        partes = texto.split()
        if len(partes) != 4 or partes[0] != 'J':
            self.get_logger().warning(f'Paquete inválido: "{texto}"')
            return
        try:
            x = int(partes[1]) / 1000.0    # el ESP32 envía -1000..1000
            y = int(partes[2]) / 1000.0
            manual = int(partes[3])
        except ValueError:
            self.get_logger().warning(f'Paquete con números inválidos: "{texto}"')
            return

        if not self.con_senal:
            self.con_senal = True
            self.get_logger().info('Señal del joystick recibida')
        self.t_ultimo_paquete = time.monotonic()
        self.publicar(x, y, manual)

    def publicar(self, x, y, manual):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [x, y]
        msg.buttons = [manual]
        self.pub.publish(msg)

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    nodo = NodoJoystick()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
