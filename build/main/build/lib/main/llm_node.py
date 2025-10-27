import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import ollama
import time
import threading
import os
import json

class LLMProcessorNode(Node):
    def __init__(self):
        super().__init__('llm_processor_node')
        self.conversation_count = 0
        self.detected_name = None
        self.current_emotion = None
        self.chat_history = []
        self.model = "llama3"
        self.history_dir = 'conversation_history' 
        self.is_ending_session = False 

        self.termination_keywords = [
            "adiós", "adios", "hasta luego", "chao", "bye", "salir", 
            "terminar", "finalizar", "despedida", "vemos"
        ]

        self.base_system_prompt = """Eres Nebu, actúa como un terapeuta(no menciones que eres) 
Características importantes:
- Eres moralmente neutral
- Responde de forma concisa y natural (máximo 1-2 oraciones)
- Mantén un tono cálido y profesional
- Responde en español
- Sé directo y útil en tus respuestas
- Sigues códigos éticos de tu profesión (terapeuta).
- Tienes una visión positiva del ser humano."""

        #Publicadores
        self.response_publisher = self.create_publisher(String, 'llm_response_text', 10) 
        self.mic_activation_publisher = self.create_publisher(Bool, 'mic_activation_state', 10) 
        
        #Suscriptores   
        self.text_subscriber = self.create_subscription(String, 'transcripcion_voz', self.text_callback, 10) 
        self.name_subscriber = self.create_subscription(String, 'person_name', self.name_callback, 10)
        self.emotion_subscriber = self.create_subscription(String, 'emocion_actual', self.emotion_callback, 10)
        self.tts_finish_subscriber = self.create_subscription(
            Bool, 
            'dialogue_continue', 
            self.dialogue_continue_callback, 
            10
        ) 

        self.get_logger().info('///////////////////Iniciando sistema////////////')
        self.initialize_system()

    def name_callback(self, msg):
        self.detected_name = msg.data.strip()
    
    def emotion_callback(self, msg):
        self.current_emotion = msg.data.strip()
    
    def dialogue_continue_callback(self, msg):
        if msg.data is True:
            if self.is_ending_session:
                self.get_logger().info('Despedida TTS completada. Publicando FALSE para desactivar sesión.')
                self.mic_activation_publisher.publish(Bool(data=False))
                self.chat_history = [] 
                self.conversation_count = 0 
                self.is_ending_session = False
            else:
                self.get_logger().info('TTS completado. Activando el micrófono para la siguiente pregunta.')
                self.mic_activation_publisher.publish(Bool(data=True)) 

    def check_termination_keywords(self, text):
        return any(kw in text.lower() for kw in self.termination_keywords)

    def is_emotion_query(self, text):
        triggers = ["mi emoción", "mi estado emocional", "cómo me siento", "cómo estoy", "mi sentimiento", "qué emoción tengo", "cómo me veo", "cuál es mi emoción"]
        return any(trigger in text.lower() for trigger in triggers)
        
    def initialize_system(self):
        try:
            ollama.chat(model=self.model, messages=[{"role": "user", "content": "Hola"}])
            self.get_logger().info('Llama3 se encuentra listo. Esperando activación por WakeWord...')
        except Exception as e:
            self.get_logger().error(f'Problemas con Ollama: {e}')

    def load_past_history_summary(self):
        if not os.path.isdir(self.history_dir):
            return None
        
        try:
            files = sorted([f for f in os.listdir(self.history_dir) if f.endswith('.json')], reverse=True)
            if not files:
                return None
            
            latest_file = os.path.join(self.history_dir, files[0])
            with open(latest_file, 'r', encoding='utf-8') as f:
                history = json.load(f)
            
            summary_parts = []
            for item in history:
                text_snippet = item['text'][:50].replace('\n', ' ')
                summary_parts.append(f"{item['role']}: {text_snippet}...")
            
            summary = "Última conversación: " + " ".join(summary_parts)
            return summary[:500] 
            
        except Exception as e:
            self.get_logger().warn(f"Error al cargar historial pasado: {e}")
            return None

    def text_callback(self, msg):
        text = msg.data.strip()
        
        if self.conversation_count == 0 and not self.check_termination_keywords(text):
            self.conversation_count = 1 
        elif self.conversation_count > 0:
            self.conversation_count += 1
            
        self.get_logger().info(f'Entrada #{self.conversation_count}: "{text}"')
        
        if not text:
            self.get_logger().warn('Transcripción vacía recibida. Pidiendo al usuario que repita.')
            self.publish_response("Disculpa, no te he entendido. ¿Podrías repetirlo?")
            return

        if self.check_termination_keywords(text):
            self.handle_goodbye()
        elif self.is_emotion_query(text):
            self.handle_emotion_query()
        else:
            threading.Thread(target=self.process_conversation, args=(text,)).start()

    def handle_goodbye(self):
        # 1. Marca la sesión para terminación
        self.is_ending_session = True 
        
        # 2. Publica la respuesta de despedida (TTS lo procesará y luego publicará FALSE)
        text = f"{self.detected_name + ', ' if self.detected_name else ''}¡Hasta luego! Si necesitas algo más, aquí estaré."
        self.get_logger().info('Palabra clave de terminación detectada. Enviando despedida y preparando el cierre del bucle.')
        self.publish_response(text)
        
        # NOTA: La señal mic_activation_publisher.publish(Bool(data=False)) se movió a dialogue_continue_callback

    def handle_emotion_query(self):
        if self.current_emotion:
            response = f"Actualmente te sientes {self.current_emotion.lower()}."
        else:
            response = "Aún no tengo información sobre tu estado emocional."
        if self.detected_name:
            response = f"{self.detected_name}, {response}"
        self.publish_response(response)

    def process_conversation(self, user_text):
        try:
            system_prompt = self.base_system_prompt
            
            #Contexto y Memoria
            if self.detected_name: system_prompt += f" Estás conversando con {self.detected_name}."
            if self.current_emotion: system_prompt += f" El usuario actualmente se siente {self.current_emotion}."
            if self.conversation_count == 1:
                past_summary = self.load_past_history_summary()
                if past_summary:
                    system_prompt += f" Ten en cuenta esta información de sesiones anteriores: {past_summary}"

            messages = [{"role": "system", "content": system_prompt}] + self.chat_history
            messages.append({"role": "user", "content": user_text})
            
            start = time.time()
            response = ollama.chat(model=self.model, messages=messages)
            elapsed = time.time() - start
            
            if response and "message" in response and "content" in response["message"]:
                llm_response = response["message"]["content"].strip()
                
                #Gestión del historial 
                self.chat_history.append({"role": "user", "content": user_text})
                self.chat_history.append({"role": "assistant", "content": llm_response})
                if len(self.chat_history) > 10: 
                    self.chat_history = self.chat_history[-10:]

                self.get_logger().info(f'Respuesta LLM ({elapsed:.2f}s): {llm_response}')
                self.publish_response(llm_response)
            else:
                self.handle_error_response(critical=False)
        except Exception as e:
            self.get_logger().error(f'Error crítico procesando conversación con Ollama: {e}')
            self.handle_error_response(critical=True)

    def handle_error_response(self, critical=False):
        text = "Disculpa, tuve un problema procesando tu solicitud ¿Puedes repetir?"
        if critical:
             text = "Mi servidor de lógica ha fallado. Por favor, reinicia mi sistema."
        self.publish_response(text)

    def publish_response(self, text):
        self.response_publisher.publish(String(data=text))
        self.get_logger().info(f'Respuesta de texto enviada al TTS: "{text}"')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = LLMProcessorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()