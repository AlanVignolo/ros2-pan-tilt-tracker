import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class NodoSelector(Node):
    """Elige quién mueve la cámara: el controlador automático o el joystick."""

    def __init__(self):
        super().__init__('nodo_selector')

        # Velocidad con la palanca a fondo, en grados por segundo.
        self.declare_parameter('velocidad_max', 60.0)
        # Signos de cada eje.
        self.declare_parameter('signo_pan', -1.0)
        self.declare_parameter('signo_tilt', -1.0)
        # Mismos límites que el controlador.
        self.declare_parameter('pan_min', -85.0)
        self.declare_parameter('pan_max', 85.0)
        self.declare_parameter('tilt_min', 0.0)
        self.declare_parameter('tilt_max', 180.0)
        self.declare_parameter('pan_inicial', 0.0)
        self.declare_parameter('tilt_inicial', 90.0)

        self.velocidad_max = self.get_parameter('velocidad_max').value
        self.signo_pan = self.get_parameter('signo_pan').value
        self.signo_tilt = self.get_parameter('signo_tilt').value
        self.pan_min = self.get_parameter('pan_min').value
        self.pan_max = self.get_parameter('pan_max').value
        self.tilt_min = self.get_parameter('tilt_min').value
        self.tilt_max = self.get_parameter('tilt_max').value

        # Último ángulo enviado a los motores, venga de donde venga.
        self.pan = self.get_parameter('pan_inicial').value
        self.tilt = self.get_parameter('tilt_inicial').value

        self.modo_manual = False
        self.palanca_x = 0.0   # último valor de la palanca, de -1.0 a 1.0
        self.palanca_y = 0.0

        self.periodo = 0.05    # cada cuánto se integra la palanca, en segundos

        self.pub = self.create_publisher(Float64MultiArray, '/cmd_angulos', 10)
        self.create_subscription(Float64MultiArray, '/cmd_auto', self.recibir_auto, 10)
        self.create_subscription(Joy, '/joy', self.recibir_joy, 10)
        self.create_timer(self.periodo, self.mover_manual)

        self.get_logger().info('Modo automático')

    def recibir_auto(self, msg):
        """Órdenes del controlador: solo pasan en modo automático."""
        if self.modo_manual or len(msg.data) != 2:
            return
        self.pan, self.tilt = msg.data[0], msg.data[1]
        self.publicar()

    def recibir_joy(self, msg):
        """Guarda la palanca y cambia de modo según el interruptor."""
        if len(msg.axes) >= 2:
            self.palanca_x, self.palanca_y = msg.axes[0], msg.axes[1]

        manual = len(msg.buttons) >= 1 and msg.buttons[0] == 1
        if manual != self.modo_manual:
            self.modo_manual = manual
            self.get_logger().info('Modo manual' if manual else 'Modo automático')

    def mover_manual(self):
        """En modo manual, suma a los ángulos un paso proporcional a la palanca."""
        if not self.modo_manual:
            return
        if self.palanca_x == 0.0 and self.palanca_y == 0.0:
            return   # palanca suelta: no se manda nada, la cámara se queda donde está

        paso = self.velocidad_max * self.periodo   # grados por ciclo con la palanca a fondo
        self.pan += self.signo_pan * paso * self.palanca_x
        self.tilt += self.signo_tilt * paso * self.palanca_y
        self.publicar()

    def publicar(self):
        # Se limita antes de publicar, por el mismo motivo que en el controlador:
        # si no, el ángulo guardado podría crecer más allá de lo que el motor puede.
        self.pan = min(max(self.pan, self.pan_min), self.pan_max)
        self.tilt = min(max(self.tilt, self.tilt_min), self.tilt_max)
        msg = Float64MultiArray()
        msg.data = [self.pan, self.tilt]
        self.pub.publish(msg)


def main():
    rclpy.init()
    nodo = NodoSelector()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()