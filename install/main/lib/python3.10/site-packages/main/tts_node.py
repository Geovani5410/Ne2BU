import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading
import requests
import pygame
import io

class TTSElevenLabsNode(Node):
    def __init__(self):
        super().__init__('tts_elevenlabs_node')

        self.API_KEY = "sk_9528eb75405a5d1f1db4985b355c71526bc8d6aab4fb14cd" 
        self.VOICE_ID = "7piC4m7q8WrpEAnMj5xC" 
        self.TTS_URL = f"https://api.elevenlabs.io/v1/text-to-speech/{self.VOICE_ID}"
        self.response_subscriber = self.create_subscription(
            String, 
            'llm_response_text', 
            self.response_callback, 
            10
        )
        
        #PUblicador
        self.dialogue_continue_publisher = self.create_publisher(Bool, 'dialogue_continue', 10)
        
        try:
            pygame.mixer.init()
            self.get_logger().info('//////////iniciando sistema///////////////////')
        except Exception as e:
            self.get_logger().error(f'Error al inicializar Pygame: {e}')

    def text_to_speech(self, text):
        try:
            headers = {
                "Accept": "audio/mpeg",
                "Content-Type": "application/json",
                "xi-api-key": self.API_KEY
            }
            data = {
                "text": text,
                "model_id": "eleven_monolingual_v1", 
                "voice_settings": {
                    "stability": 0.5,
                    "similarity_boost": 0.75
                }
            }
            response = requests.post(self.TTS_URL, json=data, headers=headers)
            
            if response.status_code == 200:
                self.get_logger().info('Audio recibido.')
                return response.content
            else:
                self.get_logger().error(f"Error con ElevenLabs: Código {response.status_code}. Respuesta: {response.text}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error en la llamada TTS: {e}")
            return None

    def play_audio(self, audio_data):
        try:
            audio_io = io.BytesIO(audio_data)
            pygame.mixer.music.load(audio_io)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.wait(100)
            self.get_logger().info('----------------Reproducción finalizada.---------------')
        except Exception as e:
            self.get_logger().error(f' Error al reproducir audio: {e}')
            
    def signal_dialogue_continue(self):
        self.dialogue_continue_publisher.publish(Bool(data=True))
        self.get_logger().info('Señal de TTS completo (dialogue_continue) enviada.')

    def response_callback(self, msg):
        text = msg.data.strip()
        self.get_logger().info(f'⬇Texto recibido: "{text}"')
        threading.Thread(target=self._synthesize_and_play, args=(text,)).start()

    def _synthesize_and_play(self, text):
        audio = self.text_to_speech(text)
        
        if audio:
            self.play_audio(audio)
        
        self.signal_dialogue_continue()

    def destroy_node(self):
        self.get_logger().info(' Cerrando Pygame...')
        pygame.mixer.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TTSElevenLabsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()