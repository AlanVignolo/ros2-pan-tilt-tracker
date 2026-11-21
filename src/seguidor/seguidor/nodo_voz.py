import json
import queue

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from seguidor_interfaces.srv import SetTarget
import sounddevice
import vosk


# Palabras que se reconocen y la clase de YOLO (en inglés) que corresponde a cada una.
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


class NodoVoz(Node):
    """Escucha el micrófono y, al reconocer un objeto, pide seguirlo con /set_target."""

    def __init__(self):
        super().__init__('nodo_voz')

        self.declare_parameter('modelo', '/home/raspberryAlan/modelos/vosk-model-small-es-0.42')
        # Parte del nombre del microfono
        self.declare_parameter('dispositivo', 'USB Composite')
        ruta_modelo = self.get_parameter('modelo').value
        dispositivo = self.get_parameter('dispositivo').value

        vosk.SetLogLevel(-1) # silencia los mensajes LOG de Vosk
        modelo = vosk.Model(ruta_modelo)

        # Se usa la frecuencia propia del micrófono: Vosk convierte internamente
        # a la del modelo, y así no dependemos de que el micrófono soporte 16 kHz.
        info = sounddevice.query_devices(dispositivo, 'input')
        tasa = int(info['default_samplerate'])

        # Gramática cerrada: solo estas palabras, más [unk] para todo lo demás.
        gramatica = json.dumps(list(VOCABULARIO.keys()) + ['[unk]'])
        self.reconocedor = vosk.KaldiRecognizer(modelo, tasa, gramatica)

        # Cola para pasar el audio del hilo de PortAudio al hilo de ROS.
        self.cola = queue.Queue()
        self.flujo = sounddevice.RawInputStream(
            samplerate=tasa,
            blocksize=tasa // 4,      # bloques de un cuarto de segundo
            dtype='int16',            # el formato que espera Vosk
            channels=1,
            device=dispositivo,
            callback=self.recibir_audio)
        self.flujo.start()

        # Clase que está buscando el detector, venga de donde venga el cambio.
        self.clase_actual = None
        # La QoS tiene que coincidir con la del publicador para recibir el último valor.
        qos_estado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/clase_objetivo', self.recibir_clase, qos_estado)

        self.cliente = self.create_client(SetTarget, '/set_target')
        self.create_timer(0.05, self.procesar_audio)

        self.get_logger().info(
            f'Escuchando a {tasa} Hz. Palabras: {", ".join(VOCABULARIO.keys())}')

    def recibir_audio(self, datos, cuadros, tiempo, estado):
        """Corre en el hilo de PortAudio: solo guarda el bloque en la cola."""
        self.cola.put(bytes(datos))

    def recibir_clase(self, msg):
        """El detector anunció la clase que está buscando."""
        self.clase_actual = msg.data

    def procesar_audio(self):
        """Corre en el timer de ROS: pasa el audio de la cola al reconocedor."""
        while not self.cola.empty():
            # AcceptWaveform devuelve True cuando detecta el final de una frase
            # (un silencio después de hablar). Recién ahí hay un resultado completo.
            if self.reconocedor.AcceptWaveform(self.cola.get()):
                texto = json.loads(self.reconocedor.Result())['text']
                if texto:
                    self.interpretar(texto)

    def interpretar(self, texto):
        """Busca en lo escuchado la primera palabra del vocabulario."""
        for palabra in texto.split():
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
        futuro = self.cliente.call_async(pedido) # manda el pedido y sigue
        futuro.add_done_callback(self.respuesta_recibida)  # se ejecuta al llegar la respuesta

    def respuesta_recibida(self, futuro):
        respuesta = futuro.result()
        if respuesta.exito:
            self.get_logger().info(respuesta.mensaje)
        else:
            self.get_logger().warning(f'No se pudo cambiar: {respuesta.mensaje}')

    def destroy_node(self):
        self.flujo.stop()    # deja de leer el micrófono
        self.flujo.close()
        super().destroy_node()


def main():
    rclpy.init()
    nodo = NodoVoz()
    try:
        rclpy.spin(nodo)
    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()