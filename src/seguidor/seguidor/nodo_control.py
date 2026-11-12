from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2D


class NodoControl(Node):
    """Mueve la cámara para mantener el objeto detectado en el centro de la imagen."""

    def __init__(self):
        super().__init__('nodo_control')

        self.declare_parameter('ancho_imagen', 640)  # Resolución real de la cámara
        self.declare_parameter('alto_imagen', 480)
        self.declare_parameter('fov_horizontal', 60.0)  # Campo de visión en grados
        self.declare_parameter('fov_vertical', 45.0)
        self.declare_parameter('ganancia', 0.4)  # Kp: fracción del error corregida por cuadro
        self.declare_parameter('zona_muerta', 2.0)  # Grados de error tolerados, evita jitter
        self.declare_parameter('signo_pan', -1.0)  # -1: con este montaje, el stepper gira al revés que el error en la imagen
        self.declare_parameter('signo_tilt', -1.0)  # -1: con este montaje, el servo se mueve al revés que el error en la imagen
        self.declare_parameter('pan_min', -85.0)  # Límites mecánicos del pan-tilt
        self.declare_parameter('pan_max', 85.0)
        self.declare_parameter('tilt_min', 0.0)
        self.declare_parameter('tilt_max', 180.0)
        self.declare_parameter('pan_inicial', 0.0)
        self.declare_parameter('tilt_inicial', 90.0)

        self.ancho = self.get_parameter('ancho_imagen').value
        self.alto = self.get_parameter('alto_imagen').value
        self.fov_h = self.get_parameter('fov_horizontal').value
        self.fov_v = self.get_parameter('fov_vertical').value
        self.ganancia = self.get_parameter('ganancia').value
        self.zona_muerta = self.get_parameter('zona_muerta').value
        self.signo_pan = self.get_parameter('signo_pan').value
        self.signo_tilt = self.get_parameter('signo_tilt').value
        self.pan_min = self.get_parameter('pan_min').value
        self.pan_max = self.get_parameter('pan_max').value
        self.tilt_min = self.get_parameter('tilt_min').value
        self.tilt_max = self.get_parameter('tilt_max').value

        # El último ángulo que mandamos. Arranca en la posición inicial.
        self.pan = self.get_parameter('pan_inicial').value
        self.tilt = self.get_parameter('tilt_inicial').value

        self.add_on_set_parameters_callback(self.al_cambiar_parametros)  # Tuning en caliente

        self.pub = self.create_publisher(Float64MultiArray, '/cmd_angulos', 10)
        self.create_subscription(Detection2D, '/deteccion', self.corregir, 10)

    def al_cambiar_parametros(self, parametros):
        for p in parametros:
            if p.name in ('ganancia', 'zona_muerta', 'signo_pan', 'signo_tilt'):
                setattr(self, p.name, p.value)  # El nombre del parámetro coincide con el atributo
        return SetParametersResult(successful=True)

    def corregir(self, det):
        # Error en píxeles respecto del centro de la imagen
        error_x = det.bbox.center.position.x - self.ancho / 2
        error_y = det.bbox.center.position.y - self.alto / 2

        # Píxeles -> grados (aproximación lineal del FOV)
        error_pan = error_x * self.fov_h / self.ancho
        error_tilt = error_y * self.fov_v / self.alto

        # Control proporcional incremental sobre la última posición enviada
        if abs(error_pan) > self.zona_muerta:
            self.pan += self.signo_pan * self.ganancia * error_pan
        if abs(error_tilt) > self.zona_muerta:
            self.tilt += self.signo_tilt * self.ganancia * error_tilt

        self.pan = min(max(self.pan, self.pan_min), self.pan_max)  # Saturación a límites
        self.tilt = min(max(self.tilt, self.tilt_min), self.tilt_max)

        msg = Float64MultiArray()
        msg.data = [self.pan, self.tilt]  # Formato que espera nodo_puente_serie
        self.pub.publish(msg)


def main():
    rclpy.init()
    nodo = NodoControl()
    try:
        rclpy.spin(nodo)  # Bloquea y despacha callbacks hasta el shutdown
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()