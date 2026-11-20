import time

from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from seguidor_interfaces.action import Buscar
from rcl_interfaces.msg import SetParametersResult
import rclpy
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

        # Velocidad del barrido si el objetivo de la acción no indica una (grados/s).
        self.declare_parameter('velocidad_busqueda', 30.0)

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
        self.velocidad_busqueda = self.get_parameter('velocidad_busqueda').value

        # El último ángulo que mandamos. Arranca en la posición inicial.
        self.pan = self.get_parameter('pan_inicial').value
        self.tilt = self.get_parameter('tilt_inicial').value

        self.add_on_set_parameters_callback(self.al_cambiar_parametros)  # Tuning en caliente

        # Publica en /cmd_auto: el selector decide si llega a los motores.
        self.pub = self.create_publisher(Float64MultiArray, '/cmd_auto', 10)
        self.create_subscription(Detection2D, '/deteccion', self.corregir, 10)
        # Escucha lo que realmente se mandó a los motores, venga del controlador
        # o del joystick, para continuar desde ahí al volver a modo automático.
        self.create_subscription(Float64MultiArray, '/cmd_angulos', self.sincronizar, 10)

        self.buscando = False     # True mientras corre una búsqueda
        self.encontrado = False   # lo pone en True corregir() cuando llega una detección

        # Servidor de la acción /buscar. Va en un grupo propio (Reentrant) para que
        # su bucle pueda correr en paralelo con los callbacks de detección.
        self.servidor_buscar = ActionServer(
            self, Buscar, '/buscar',
            execute_callback=self.ejecutar_busqueda,
            cancel_callback=self.aceptar_cancelacion,
            callback_group=ReentrantCallbackGroup())

    def al_cambiar_parametros(self, parametros):
        for p in parametros:
            if p.name in ('ganancia', 'zona_muerta', 'signo_pan', 'signo_tilt'):
                setattr(self, p.name, p.value)  # El nombre del parámetro coincide con el atributo
        return SetParametersResult(successful=True)

    def corregir(self, det):
        # Si hay una búsqueda en curso, esta detección la termina con éxito.
        if self.buscando:
            self.encontrado = True

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

    def sincronizar(self, msg):
        """Actualiza el ángulo propio con el último enviado a los motores."""
        if len(msg.data) == 2:
            self.pan, self.tilt = msg.data[0], msg.data[1]

    def aceptar_cancelacion(self, goal_handle):
        """Acepta siempre los pedidos de cancelación.

        Si no se define este callback, rclpy rechaza todas las cancelaciones.
        """
        return CancelResponse.ACCEPT

    def ejecutar_busqueda(self, goal_handle):
        """Barre el pan de un extremo al otro hasta que llegue una detección."""
        velocidad = goal_handle.request.velocidad
        if velocidad <= 0.0:
            velocidad = self.velocidad_busqueda

        periodo = 0.05                 # un paso cada 50 ms
        paso = velocidad * periodo     # grados por paso
        pan = self.pan                 # variable propia del bucle, arranca donde está la cámara
        direccion = 1.0                # primero hacia pan_max, después hacia pan_min

        # Recorrido completo: hasta pan_max, y de ahí hasta pan_min.
        total = (self.pan_max - pan) + (self.pan_max - self.pan_min)
        recorrido = 0.0

        self.encontrado = False
        self.buscando = True
        self.get_logger().info(f'Buscando a {velocidad:.0f} grados por segundo')

        resultado = Buscar.Result()
        avance = Buscar.Feedback()

        while True:
            # 1. ¿Pidieron cancelar?
            if goal_handle.is_cancel_requested:
                self.buscando = False
                goal_handle.canceled()
                self.get_logger().info('Búsqueda cancelada')
                resultado.encontrado = False
                resultado.pan = self.pan
                return resultado

            # 2. ¿Llegó una detección? (la marca corregir() desde otro hilo)
            if self.encontrado:
                self.buscando = False
                goal_handle.succeed()
                self.get_logger().info(f'Encontrado en pan {self.pan:.1f}')
                resultado.encontrado = True
                resultado.pan = self.pan
                return resultado

            # 3. ¿Recorrió todo sin encontrar?
            if recorrido >= total:
                self.buscando = False
                goal_handle.abort()
                self.get_logger().warning('Búsqueda terminada sin encontrar nada')
                resultado.encontrado = False
                resultado.pan = self.pan
                return resultado

            # 4. Dar un paso del barrido
            pan += direccion * paso
            recorrido += paso
            if pan >= self.pan_max:
                pan = self.pan_max
                direccion = -1.0   # llegó al extremo: dar la vuelta
            pan = max(pan, self.pan_min)

            self.pan = pan
            msg = Float64MultiArray()
            msg.data = [self.pan, self.tilt]
            self.pub.publish(msg)

            avance.pan = pan
            avance.progreso = min(recorrido / total, 1.0)
            goal_handle.publish_feedback(avance)

            time.sleep(periodo)


def main():
    rclpy.init()
    nodo = NodoControl()
    # Ejecutor de varios hilos: permite correr la búsqueda y las detecciones en paralelo.
    ejecutor = MultiThreadedExecutor()
    ejecutor.add_node(nodo)
    try:
        ejecutor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()