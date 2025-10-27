import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
import threading
import os
import json
import sounddevice as sd
from vosk import Model, KaldiRecognizer 

from ament_index_python.packages import get_package_share_directory

# Configuracion inicial
WAKE_WORD = "hola" 
SAMPLE_RATE = 16000 
PACKAGE_NAME = 'main' 

try:
    SHARE_DIR = get_package_share_directory(PACKAGE_NAME)
    MODEL_DIR_VOSK = os.path.join(SHARE_DIR, 'vosk_model_es')
except Exception as e:
    # Ruta de respaldo si get_package_share_directory falla
    MODEL_DIR_VOSK = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vosk_model_es') 

class WakeNode(Node):
    def __init__(self):
        super().__init__('wake_node')
        
        # Publicador del estado de activación del micrófono (para VOZ_NODE, LLM_NODE y BRIDGE_NODE)
        self.wake_publisher = self.create_publisher(Bool, 'mic_activation_state', 10)
        
        # Suscripción al LLM para saber cuándo desactivarse (volver a la escucha de la palabra clave)
        self.deactivation_sub = self.create_subscription(
            Bool, 
            'mic_activation_state', 
            self.activation_state_callback, 
            10
        )

        self.get_logger().info(f'LISTO ({WAKE_WORD.upper()}) ////Iniciando sistema////')
        
        self.mic_activated = False
        self.listening_for_wake = True 
        
        # ELIMINADA: La inicialización serial

        # Iniciar VOSK
        if self._init_vosk():
            threading.Thread(target=self._start_listening).start()

    # ELIMINADA: La función _init_serial
    # ELIMINADA: La función send_serial_command

    def _init_vosk(self):
        if not os.path.isdir(MODEL_DIR_VOSK):
             self.get_logger().error(
                 f"Error: El directorio del modelo VOSK no se encuentra en {MODEL_DIR_VOSK}."
             )
             return False
        
        try:
            self.model = Model(MODEL_DIR_VOSK)
            grammar = json.dumps([WAKE_WORD, "[unk]"])
        
            self.recognizer = KaldiRecognizer(self.model, SAMPLE_RATE, grammar)
            self.get_logger().info(" VOSK se encuentra listo")
            return True
        except Exception as e:
            self.get_logger().error(f"Error al iniciar VOSK: {e}")
            return False

    # Controla el estado del sistema. Si recibe 'False', vuelve a la escucha
    def activation_state_callback(self, msg):
        # Se asegura de que solo reaccione a una señal de desactivación (False)
        # y que el sistema ya no esté esperando la palabra clave.
        if msg.data is False and not self.listening_for_wake:
            self.mic_activated = False
            self.listening_for_wake = True
            self.get_logger().info("Regresando al sistema de espera de la palabra clave.")
            
            # La señal 'False' ya está siendo publicada por el LLM en este punto.
            # No es necesario volver a publicarla aquí.
            
            # ELIMINADA: self.send_serial_command('WAIT')
            # Se asegura de que el estado inicial de no activación esté claro para todos los suscriptores
            self.publish_mic_state(False) 
            
            threading.Thread(target=self._start_listening).start()

    def _start_listening(self):
        self.get_logger().info("Esperando la palabra clave... (microfono activo)")
        
        def callback(indata, frames, time, status):
            if status:
                self.get_logger().warn(f"Estado de audio: {status}")
            
            if self.listening_for_wake:
                if self.recognizer.AcceptWaveform(indata.tobytes()):
                    result = json.loads(self.recognizer.Result())
                    text = result.get('text', '').strip()

                    # Comprueba la palabra clave
                    if text == WAKE_WORD:
                        self.get_logger().info(f"/////////////////PALABRA CLAVE: '{WAKE_WORD}'/////////////////")
                        self.activate_transcriber()
                        raise sd.CallbackStop # Detiene la captura de audio

        try:
            with sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='int16', callback=callback):
                # Mantener el hilo de escucha activo
                while self.listening_for_wake and self.recognizer:
                    time.sleep(0.1) 

        except sd.CallbackStop:
            self.get_logger().info("Stream de audio detenido tras detección de palabra clave.")
        except Exception as e:
            self.get_logger().error(f"Error en el stream de audio VOSK: {e}")
            self.listening_for_wake = False


    def activate_transcriber(self):
        # Activa el voz_node e inhabilita la escucha del WakeNode
        if not self.mic_activated:
            self.mic_activated = True
            self.listening_for_wake = False 
            
            # Publica la señal de activación (True) para VOZ_NODE y BRIDGE_NODE
            self.publish_mic_state(True) 
            
            # ELIMINADA: self.send_serial_command('ACTIVE')
            
            self.get_logger().info("Micrófono activado. voz_node (Whisper) está activo")

    def publish_mic_state(self, state):
        # Publica el estado del micrófono
        msg = Bool()
        msg.data = state
        self.wake_publisher.publish(msg)

    def destroy_node(self):
        # ELIMINADA: La lógica de cierre del puerto serial
        self.get_logger().info('wake_node cerrado.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WakeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()