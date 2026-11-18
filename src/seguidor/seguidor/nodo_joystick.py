import socket
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


class NodoJoystick(Node):
    """Recibe el joystick del ESP32 por UDP y lo publica en /joy."""

    def __init__(self):
        super().__init__('nodo_joystick')

        self.declare_parameter('puerto', 5005)
        # Si no llega ningún paquete en este tiempo, se considera perdida la señal.
        self.declare_parameter('timeout', 0.5)
        puerto = self.get_parameter('puerto').value
        self.timeout = self.get_parameter('timeout').value

        # Socket UDP: puerta de red por donde llegan los paquetes
        # AF_INET = direcciones IPv4. SOCK_DGRAM = UDP (paquetes sueltos, sin conexión).
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # bind: escuchar en ese puerto. '0.0.0.0' = en todas las placas de red de la Raspberry.
        self.sock.bind(('0.0.0.0', puerto))
        # No bloqueante: si no hay paquetes, recvfrom no espera, lanza BlockingIOError.
        # Así el timer nunca se queda trabado esperando.
        self.sock.setblocking(False)

        self.pub = self.create_publisher(Joy, '/joy', 10)

        self.t_ultimo_paquete = 0.0 # hora del ultimo paquete recibido
        self.con_senal = False # si en este momento llegan paquetes

        self.create_timer(0.01, self.leer_socket)
        self.get_logger().info(f'Escuchando el joystick en el puerto UDP {puerto}')

    def leer_socket(self):
        """Lee todos los paquetes que llegaron y vigila la señal."""
        while True:
            try:
                datos, _ = self.sock.recvfrom(64)   # hasta 64 bytes, de sobra para "J x y m"
            except BlockingIOError:
                break   # no quedan paquetes por leer
            self.procesar_paquete(datos.decode('ascii', errors='replace').strip())

        # Vigilante: si dejaron de llegar paquetes, publicar todo en cero una vez.
        if self.con_senal and time.monotonic() - self.t_ultimo_paquete > self.timeout:
            self.con_senal = False
            self.get_logger().warning('Sin señal del joystick: ejes en cero')
            self.publicar(0.0, 0.0, 0)

    def procesar_paquete(self, texto):
        """Interpreta "J <x> <y> <manual>" y lo publica."""
        partes = texto.split()
        if len(partes) != 4 or partes[0] != 'J':
            self.get_logger().warning(f'Paquete inválido: "{texto}"')
            return
        try:
            x = int(partes[1]) / 1000.0   # de -1000..1000 a -1.0..1.0
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
        msg.header.stamp = self.get_clock().now().to_msg()   # hora de la publicación
        msg.axes = [x, y]
        msg.buttons = [manual]
        self.pub.publish(msg)

    def destroy_node(self):
        self.sock.close()   # libera el puerto al apagar el nodo
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