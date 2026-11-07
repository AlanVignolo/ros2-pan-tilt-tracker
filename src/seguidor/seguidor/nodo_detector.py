import time

from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose


class NodoDetector(Node):
    """Busca un objeto con YOLO en cada imagen y publica dónde está."""

    def __init__(self):
        super().__init__('nodo_detector')

        # Parámetros configurables desde launch o con `ros2 param set`
        self.declare_parameter('modelo', 'yolov8n.pt')
        self.declare_parameter('clase_objetivo', 'cat')
        self.declare_parameter('umbral_confianza', 0.5)
        self.declare_parameter('tamano_entrada', 320)  # Resolución de inferencia (px)
        self.declare_parameter('tiempo_perdida', 0.5)  # Segundos sin detección = objeto perdido

        self.modelo = YOLO(self.get_parameter('modelo').value)
        self.umbral = self.get_parameter('umbral_confianza').value
        self.tamano = self.get_parameter('tamano_entrada').value
        self.tiempo_perdida = self.get_parameter('tiempo_perdida').value

        self.detectando = False
        self.t_ultima_deteccion = 0.0

        self.clase = None
        self.id_clase = None
        self.cambiar_clase(self.get_parameter('clase_objetivo').value)

        self.add_on_set_parameters_callback(self.al_cambiar_parametros)  # Cambios en caliente

        self.bridge = CvBridge()  # Conversión sensor_msgs/Image <-> OpenCV
        self.pub_deteccion = self.create_publisher(Detection2D, '/deteccion', 10)
        self.pub_imagen = self.create_publisher(Image, '/deteccion/imagen', 10)

        # BEST_EFFORT + depth=1: se procesa siempre el cuadro más reciente, sin cola
        qos_imagen = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/image_raw', self.procesar_imagen, qos_imagen)

        self.cuadros = 0
        self.t_reporte = time.monotonic()

    def cambiar_clase(self, nombre):
        """Cambia el objeto a buscar. Devuelve False si la clase no existe."""
        for numero, nombre_modelo in self.modelo.names.items():  # Mapa id -> nombre de COCO
            if nombre_modelo == nombre:
                self.clase = nombre
                self.id_clase = numero
                self.detectando = False
                self.get_logger().info(f'Buscando "{nombre}"')
                return True
        self.get_logger().warning(f'La clase "{nombre}" no existe en el modelo')
        return False

    def al_cambiar_parametros(self, parametros):
        for p in parametros:
            if p.name == 'clase_objetivo':
                if not self.cambiar_clase(p.value):
                    return SetParametersResult(  # Rechaza el cambio de parámetro
                        successful=False, reason=f'clase "{p.value}" desconocida')
            elif p.name == 'umbral_confianza':
                self.umbral = p.value
        return SetParametersResult(successful=True)

    def procesar_imagen(self, msg):
        if self.id_clase is None:
            return

        imagen = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        resultado = self.modelo(  # Inferencia filtrada a la clase objetivo
            imagen, imgsz=self.tamano, conf=self.umbral,
            classes=[self.id_clase], verbose=False)[0]
        ahora = time.monotonic()

        if len(resultado.boxes) > 0:
            mejor = int(resultado.boxes.conf.argmax())  # Detección de mayor confianza
            cx, cy, ancho, alto = resultado.boxes.xywh[mejor].tolist()  # Centro y tamaño (px)
            confianza = float(resultado.boxes.conf[mejor])

            if not self.detectando:
                self.get_logger().info(
                    f'Encontré "{self.clase}" (confianza {confianza:.2f})')
                self.detectando = True
            self.t_ultima_deteccion = ahora

            deteccion = Detection2D()
            deteccion.header = msg.header  # Conserva el timestamp y frame de la cámara
            deteccion.bbox.center.position.x = cx
            deteccion.bbox.center.position.y = cy
            deteccion.bbox.size_x = ancho
            deteccion.bbox.size_y = alto

            hipotesis = ObjectHypothesisWithPose()
            hipotesis.hypothesis.class_id = self.clase
            hipotesis.hypothesis.score = confianza
            deteccion.results.append(hipotesis)

            self.pub_deteccion.publish(deteccion)

        # Histéresis temporal: evita reportar pérdida por un cuadro aislado sin detección
        elif self.detectando and ahora - self.t_ultima_deteccion > self.tiempo_perdida:
            self.get_logger().warning(f'Perdí "{self.clase}"')
            self.detectando = False

        if self.pub_imagen.get_subscription_count() > 0:  # Solo dibuja si alguien escucha
            anotada = self.bridge.cv2_to_imgmsg(resultado.plot(), encoding='bgr8')
            anotada.header = msg.header
            self.pub_imagen.publish(anotada)

        self.cuadros += 1

        if ahora - self.t_reporte >= 5.0:  # Reporte de FPS cada 5 s
            fps = self.cuadros / (ahora - self.t_reporte)
            self.get_logger().info(f'Procesando {fps:.1f} cuadros por segundo')
            self.cuadros = 0
            self.t_reporte = ahora


def main():
    rclpy.init()
    nodo = NodoDetector()
    try:
        rclpy.spin(nodo)  # Bloquea y despacha callbacks hasta el shutdown
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
