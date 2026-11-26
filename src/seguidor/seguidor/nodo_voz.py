"""Comandos de voz offline con Vosk."""

import json
import queue

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from seguidor_interfaces.action import Buscar
from seguidor_interfaces.srv import SetTarget
import sounddevice
from std_msgs.msg import String
from std_srvs.srv import Trigger
import vosk

# Palabra reconocida -> clase COCO que usa el detector.
VOCABULARIO = {
    'gato': 'cat',
    'perro': 'dog',
    'persona': 'person',
    'taza': 'cup',
    'botella': 'bottle',
    'celular': 'cell phone',
    'silla': 'chair',
    'pelota': 'sports ball',
}

# calibrar -> /homing, buscar -> /buscar
COMANDOS = ['calibrar', 'buscar']


class NodoVoz(Node):
    """Reconoce palabras del micrófono y las traduce a /set_target, /homing y /buscar."""

    def __init__(self):
        super().__init__('nodo_voz')

        self.declare_parameter('modelo', '/home/raspberryAlan/modelos/vosk-model-small-es-0.42')
        self.declare_parameter('dispositivo', 'USB Composite')    # parte del nombre del micrófono
        ruta_modelo = self.get_parameter('modelo').value
        dispositivo = self.get_parameter('dispositivo').value

        vosk.SetLogLevel(-1)
        modelo = vosk.Model(ruta_modelo)

        # Se usa la frecuencia nativa del micrófono y Vosk remuestrea internamente.
        # Así no depende de que el micrófono soporte 16 kHz.
        info = sounddevice.query_devices(dispositivo, 'input')
        tasa = int(info['default_samplerate'])

        # Gramática cerrada: limita el reconocimiento a estas palabras y baja
        # mucho los falsos positivos con el modelo chico.
        gramatica = json.dumps(list(VOCABULARIO.keys()) + COMANDOS + ['[unk]'])
        self.reconocedor = vosk.KaldiRecognizer(modelo, tasa, gramatica)

        # El callback de audio corre en el hilo de PortAudio. La cola pasa los
        # bloques al hilo de ROS, donde se hace el reconocimiento.
        self.cola = queue.Queue()
        self.flujo = sounddevice.RawInputStream(
            samplerate=tasa,
            blocksize=tasa // 4,
            dtype='int16',
            channels=1,
            device=dispositivo,
            callback=self.recibir_audio)
        self.flujo.start()

        self.clase_actual = None
        # Misma QoS que el publicador para recibir el último valor aunque se publicó antes.
        qos_estado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/clase_objetivo', self.recibir_clase, qos_estado)

        self.cliente = self.create_client(SetTarget, '/set_target')
        self.cliente_homing = self.create_client(Trigger, '/homing')
        self.cliente_buscar = ActionClient(self, Buscar, '/buscar')

        self.create_timer(0.05, self.procesar_audio)

        self.get_logger().info(
            f'Escuchando a {tasa} Hz. Palabras: {", ".join(VOCABULARIO.keys())}. '
            f'Comandos: {", ".join(COMANDOS)}')

    def recibir_audio(self, datos, cuadros, tiempo, estado):
        self.cola.put(bytes(datos))

    def recibir_clase(self, msg):
        self.clase_actual = msg.data

    def procesar_audio(self):
        while not self.cola.empty():
            # AcceptWaveform devuelve True al detectar el silencio que cierra una frase.
            if self.reconocedor.AcceptWaveform(self.cola.get()):
                texto = json.loads(self.reconocedor.Result())['text']
                if texto:
                    self.interpretar(texto)

    def interpretar(self, texto):
        for palabra in texto.split():
            if palabra == 'calibrar':
                self.pedir_homing()
                return
            if palabra == 'buscar':
                self.pedir_busqueda()
                return
            if palabra in VOCABULARIO:
                self.pedir_clase(palabra, VOCABULARIO[palabra])
                return
        self.get_logger().debug(f'Escuché algo que no está en el vocabulario: "{texto}"')

    def pedir_clase(self, palabra, clase):
        if clase == self.clase_actual:
            self.get_logger().info(f'Escuché "{palabra}": ya estoy siguiendo "{clase}"')
            return
        if not self.cliente.service_is_ready():
            self.get_logger().warning('El servicio /set_target no está disponible')
            return
        self.get_logger().info(f'Escuché "{palabra}": pido seguir "{clase}"')
        pedido = SetTarget.Request()
        pedido.clase = clase
        futuro = self.cliente.call_async(pedido)
        futuro.add_done_callback(self.respuesta_recibida)

    def respuesta_recibida(self, futuro):
        respuesta = futuro.result()
        if respuesta.exito:
            self.get_logger().info(respuesta.mensaje)
        else:
            self.get_logger().warning(f'No se pudo cambiar: {respuesta.mensaje}')

    def pedir_homing(self):
        if not self.cliente_homing.service_is_ready():
            self.get_logger().warning('El servicio /homing no está disponible')
            return
        self.get_logger().info('Escuché "calibrar": pido el homing')
        futuro = self.cliente_homing.call_async(Trigger.Request())
        futuro.add_done_callback(self.respuesta_homing)

    def respuesta_homing(self, futuro):
        # El puente responde al iniciar el homing, no al terminarlo.
        respuesta = futuro.result()
        if respuesta.success:
            self.get_logger().info(respuesta.message)
        else:
            self.get_logger().warning(f'No se pudo hacer el homing: {respuesta.message}')

    def pedir_busqueda(self):
        if not self.cliente_buscar.server_is_ready():
            self.get_logger().warning('La acción /buscar no está disponible')
            return
        self.get_logger().info('Escuché "buscar": pido un barrido')
        objetivo = Buscar.Goal()
        objetivo.velocidad = 0.0    # 0 usa la velocidad por defecto del controlador
        futuro = self.cliente_buscar.send_goal_async(objetivo)
        futuro.add_done_callback(self.busqueda_aceptada)

    def busqueda_aceptada(self, futuro):
        manejador = futuro.result()
        if not manejador.accepted:
            self.get_logger().warning('El controlador rechazó el barrido')
            return
        manejador.get_result_async().add_done_callback(self.busqueda_terminada)

    def busqueda_terminada(self, futuro):
        resultado = futuro.result().result
        if resultado.encontrado:
            self.get_logger().info(f'Barrido: encontrado en pan {resultado.pan:.1f}°')
        else:
            self.get_logger().info('Barrido: recorrió todo y no lo encontró')

    def destroy_node(self):
        self.flujo.stop()
        self.flujo.close()
        super().destroy_node()


def main():
    rclpy.init()
    nodo = NodoVoz()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
