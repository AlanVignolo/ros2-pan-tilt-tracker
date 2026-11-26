"""Control pan/tilt que mantiene centrado el objeto detectado."""

import time

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from seguidor_interfaces.action import Buscar
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2D


class NodoControl(Node):
    """Lleva el centro de la detección al centro de la imagen y atiende /buscar."""

    def __init__(self):
        super().__init__('nodo_control')

        self.declare_parameter('ancho_imagen', 640)
        self.declare_parameter('alto_imagen', 480)
        self.declare_parameter('fov_horizontal', 60.0)    # grados
        self.declare_parameter('fov_vertical', 45.0)      # grados
        self.declare_parameter('ganancia', 0.4)           # fracción del error corregida por cuadro
        self.declare_parameter('zona_muerta', 2.0)        # grados de error que no se corrigen
        # Los signos dependen del montaje mecánico. En el prototipo actual
        # ambos ejes se mueven al revés que el error medido en la imagen.
        self.declare_parameter('signo_pan', -1.0)
        self.declare_parameter('signo_tilt', -1.0)
        self.declare_parameter('pan_min', -85.0)
        self.declare_parameter('pan_max', 85.0)
        self.declare_parameter('tilt_min', 0.0)
        self.declare_parameter('tilt_max', 180.0)
        self.declare_parameter('pan_inicial', 0.0)
        self.declare_parameter('tilt_inicial', 90.0)
        self.declare_parameter('velocidad_busqueda', 30.0)  # grados/s si el goal no la indica

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

        # Último ángulo comandado. El control es incremental sobre este valor.
        self.pan = self.get_parameter('pan_inicial').value
        self.tilt = self.get_parameter('tilt_inicial').value

        self.add_on_set_parameters_callback(self.al_cambiar_parametros)

        # Este nodo no manda directo a los motores: publica en /cmd_auto y el
        # selector decide si esa orden pasa o si manda el joystick.
        self.pub = self.create_publisher(Float64MultiArray, '/cmd_auto', 10)
        self.create_subscription(Detection2D, '/deteccion', self.corregir, 10)
        # /cmd_angulos es lo que realmente se envió a los motores. Seguirlo evita
        # un salto al volver de modo manual a automático.
        self.create_subscription(Float64MultiArray, '/cmd_angulos', self.sincronizar, 10)

        self.buscando = False
        self.encontrado = False

        # Grupo reentrante para que el bucle de la búsqueda corra en paralelo
        # con los callbacks de detección (requiere MultiThreadedExecutor).
        self.servidor_buscar = ActionServer(
            self, Buscar, '/buscar',
            execute_callback=self.ejecutar_busqueda,
            cancel_callback=self.aceptar_cancelacion,
            callback_group=ReentrantCallbackGroup())

    def al_cambiar_parametros(self, parametros):
        for p in parametros:
            if p.name in ('ganancia', 'zona_muerta', 'signo_pan', 'signo_tilt'):
                setattr(self, p.name, p.value)
        return SetParametersResult(successful=True)

    def corregir(self, det):
        if self.buscando:
            self.encontrado = True

        error_x = det.bbox.center.position.x - self.ancho / 2
        error_y = det.bbox.center.position.y - self.alto / 2

        # Conversión píxeles a grados con aproximación lineal del FOV.
        # Alcanza para un lente de 60° y evita calibrar la cámara.
        error_pan = error_x * self.fov_h / self.ancho
        error_tilt = error_y * self.fov_v / self.alto

        if abs(error_pan) > self.zona_muerta:
            self.pan += self.signo_pan * self.ganancia * error_pan
        if abs(error_tilt) > self.zona_muerta:
            self.tilt += self.signo_tilt * self.ganancia * error_tilt

        # Saturar acá y no solo en el firmware, para que el ángulo guardado
        # no siga creciendo más allá de lo que el eje puede alcanzar.
        self.pan = min(max(self.pan, self.pan_min), self.pan_max)
        self.tilt = min(max(self.tilt, self.tilt_min), self.tilt_max)

        self.pub.publish(Float64MultiArray(data=[self.pan, self.tilt]))

    def sincronizar(self, msg):
        if len(msg.data) == 2:
            self.pan, self.tilt = msg.data[0], msg.data[1]

    def aceptar_cancelacion(self, goal_handle):
        # Sin este callback rclpy rechaza todos los pedidos de cancelación.
        return CancelResponse.ACCEPT

    def ejecutar_busqueda(self, goal_handle):
        """Barre el pan hasta pan_max y luego hasta pan_min, o hasta detectar el objeto."""
        velocidad = goal_handle.request.velocidad
        if velocidad <= 0.0:
            velocidad = self.velocidad_busqueda

        periodo = 0.05
        paso = velocidad * periodo
        pan = self.pan
        direccion = 1.0

        total = (self.pan_max - pan) + (self.pan_max - self.pan_min)
        recorrido = 0.0

        self.encontrado = False
        self.buscando = True
        self.get_logger().info(f'Buscando a {velocidad:.0f} grados por segundo')

        resultado = Buscar.Result()
        avance = Buscar.Feedback()

        while True:
            if goal_handle.is_cancel_requested:
                self.buscando = False
                goal_handle.canceled()
                self.get_logger().info('Búsqueda cancelada')
                resultado.encontrado = False
                resultado.pan = self.pan
                return resultado

            # self.encontrado lo modifica corregir() desde otro hilo del executor.
            if self.encontrado:
                self.buscando = False
                goal_handle.succeed()
                self.get_logger().info(f'Encontrado en pan {self.pan:.1f}')
                resultado.encontrado = True
                resultado.pan = self.pan
                return resultado

            if recorrido >= total:
                self.buscando = False
                goal_handle.abort()
                self.get_logger().warning('Búsqueda terminada sin encontrar nada')
                resultado.encontrado = False
                resultado.pan = self.pan
                return resultado

            pan += direccion * paso
            recorrido += paso
            if pan >= self.pan_max:
                pan = self.pan_max
                direccion = -1.0
            pan = max(pan, self.pan_min)

            self.pan = pan
            self.pub.publish(Float64MultiArray(data=[self.pan, self.tilt]))

            avance.pan = pan
            avance.progreso = min(recorrido / total, 1.0)
            goal_handle.publish_feedback(avance)

            time.sleep(periodo)


def main():
    rclpy.init()
    nodo = NodoControl()
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
