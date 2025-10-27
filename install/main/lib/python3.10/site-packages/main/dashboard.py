import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import datetime
import time
from collections import deque
import statistics
import os

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        
        # === MÉTRICAS DE TIEMPO ===
        self.session_start_time = None
        self.wake_word_time = None
        self.user_speech_start = None
        self.user_speech_end = None
        self.llm_start_time = None
        self.llm_end_time = None
        self.tts_start_time = None
        self.tts_end_time = None
        
        # Historial de tiempos (últimas 10 interacciones)
        self.total_response_times = deque(maxlen=10)  # Desde que habla el usuario hasta que termina TTS
        self.whisper_times = deque(maxlen=10)
        self.llm_times = deque(maxlen=10)
        self.tts_times = deque(maxlen=10)
        
        # === CONTADORES DE SESIÓN ===
        self.total_sessions = 0
        self.current_session_messages = 0
        self.total_messages_ever = 0
        
        # === ESTADO DEL SISTEMA ===
        self.current_state = "IDLE"  # IDLE, WAKE_DETECTED, LISTENING, PROCESSING_LLM, SPEAKING
        self.mic_active = False
        self.last_user_text = ""
        self.last_llm_response = ""
        
        # === SUSCRIPTORES ===
        # Wake Word Detection
        self.wake_sub = self.create_subscription(
            Bool, 'mic_activation_state', self.wake_callback, 10
        )
        
        # Transcripción del Usuario
        self.transcription_sub = self.create_subscription(
            String, 'transcripcion_voz', self.transcription_callback, 10
        )
        
        # Respuesta del LLM
        self.llm_sub = self.create_subscription(
            String, 'llm_response_text', self.llm_callback, 10
        )
        
        # Fin del TTS
        self.tts_sub = self.create_subscription(
            Bool, 'dialogue_continue', self.tts_callback, 10
        )
        
        # LED State (Bridge)
        self.led_sub = self.create_subscription(
            Bool, '/micro_ros_led', self.led_callback, 10
        )
        
        # === TIMER PARA DISPLAY ===
        self.display_timer = self.create_timer(2.0, self.display_dashboard)
        
        self.get_logger().info('╔═══════════════════════════════════════════════════════╗')
        self.get_logger().info('║        NEBU DASHBOARD - Sistema Iniciado           ║')
        self.get_logger().info('╚═══════════════════════════════════════════════════════╝')
    
    # === CALLBACKS ===
    
    def wake_callback(self, msg):
        """Detecta activación/desactivación del sistema"""
        if msg.data is True:
            if self.current_state == "IDLE":
                self.wake_word_time = time.time()
                self.session_start_time = time.time()
                self.current_state = "WAKE_DETECTED"
                self.total_sessions += 1
                self.current_session_messages = 0
                self.get_logger().info('🎤 WAKE WORD DETECTADO - Nueva sesión iniciada')
            else:
                # Micrófono reactivado después de TTS
                self.current_state = "LISTENING"
                self.user_speech_start = time.time()
        
        elif msg.data is False:
            # Sesión terminada
            if self.current_state != "IDLE":
                session_duration = time.time() - self.session_start_time if self.session_start_time else 0
                self.get_logger().info(f'🔴 SESIÓN FINALIZADA - Duración: {session_duration:.1f}s, Mensajes: {self.current_session_messages}')
                self.current_state = "IDLE"
                self.session_start_time = None
    
    def transcription_callback(self, msg):
        """Usuario terminó de hablar"""
        self.last_user_text = msg.data.strip()
        if self.last_user_text:
            self.user_speech_end = time.time()
            self.llm_start_time = time.time()
            self.current_state = "PROCESSING_LLM"
            self.current_session_messages += 1
            self.total_messages_ever += 1
            
            # Calcular tiempo de Whisper
            if self.user_speech_start:
                whisper_time = self.user_speech_end - self.user_speech_start
                self.whisper_times.append(whisper_time)
                self.get_logger().info(f'⏱️  Whisper: {whisper_time:.2f}s | "{self.last_user_text[:40]}..."')
    
    def llm_callback(self, msg):
        """LLM generó respuesta"""
        self.last_llm_response = msg.data.strip()
        if self.last_llm_response:
            self.llm_end_time = time.time()
            self.tts_start_time = time.time()
            self.current_state = "SPEAKING"
            
            # Calcular tiempo del LLM
            if self.llm_start_time:
                llm_time = self.llm_end_time - self.llm_start_time
                self.llm_times.append(llm_time)
                self.get_logger().info(f'🧠 LLM: {llm_time:.2f}s | "{self.last_llm_response[:40]}..."')
    
    def tts_callback(self, msg):
        """TTS terminó de hablar"""
        if msg.data is True:
            self.tts_end_time = time.time()
            
            # Calcular tiempo del TTS
            if self.tts_start_time:
                tts_time = self.tts_end_time - self.tts_start_time
                self.tts_times.append(tts_time)
                self.get_logger().info(f'🔊 TTS: {tts_time:.2f}s')
            
            # Calcular tiempo total de respuesta
            if self.user_speech_end:
                total_time = self.tts_end_time - self.user_speech_end
                self.total_response_times.append(total_time)
                self.get_logger().info(f'⚡ TIEMPO TOTAL DE RESPUESTA: {total_time:.2f}s')
            
            self.current_state = "LISTENING"
    
    def led_callback(self, msg):
        """Estado del LED (indicador visual)"""
        # Solo para monitoreo, no afecta métricas
        pass
    
    # === DISPLAY DEL DASHBOARD ===
    
    def display_dashboard(self):
        """Muestra el dashboard cada 2 segundos"""
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("\n" + "="*70)
        print(" "*20 + "🤖 NEBU DASHBOARD 🤖")
        print("="*70)
        
        # === ESTADO ACTUAL ===
        print(f"\n📊 ESTADO ACTUAL: {self._get_state_emoji()} {self.current_state}")
        print(f"🔴 LED Estado: {'🟢 ENCENDIDO' if self.mic_active else '⚫ APAGADO'}")
        
        # === SESIÓN ACTUAL ===
        print(f"\n📈 SESIÓN ACTUAL:")
        print(f"   • Número de sesión: #{self.total_sessions}")
        print(f"   • Mensajes en esta sesión: {self.current_session_messages}")
        if self.session_start_time:
            duration = time.time() - self.session_start_time
            print(f"   • Duración de sesión: {duration:.1f}s")
        
        # === ÚLTIMA INTERACCIÓN ===
        print(f"\n💬 ÚLTIMA INTERACCIÓN:")
        print(f"   👤 Usuario: {self.last_user_text[:50] if self.last_user_text else 'N/A'}...")
        print(f"   🤖 Nebu: {self.last_llm_response[:50] if self.last_llm_response else 'N/A'}...")
        
        # === MÉTRICAS DE RENDIMIENTO ===
        print(f"\n⚡ TIEMPOS DE RESPUESTA (Promedio de últimas {len(self.total_response_times)} interacciones):")
        
        if self.whisper_times:
            avg_whisper = statistics.mean(self.whisper_times)
            print(f"   🎤 Whisper (STT): {avg_whisper:.2f}s")
        
        if self.llm_times:
            avg_llm = statistics.mean(self.llm_times)
            min_llm = min(self.llm_times)
            max_llm = max(self.llm_times)
            print(f"   🧠 LLM (Llama3): {avg_llm:.2f}s (min: {min_llm:.2f}s, max: {max_llm:.2f}s)")
        
        if self.tts_times:
            avg_tts = statistics.mean(self.tts_times)
            print(f"   🔊 TTS (ElevenLabs): {avg_tts:.2f}s")
        
        if self.total_response_times:
            avg_total = statistics.mean(self.total_response_times)
            min_total = min(self.total_response_times)
            max_total = max(self.total_response_times)
            print(f"\n   ⚡ TIEMPO TOTAL: {avg_total:.2f}s (min: {min_total:.2f}s, max: {max_total:.2f}s)")
            
            # Desglose porcentual
            if self.whisper_times and self.llm_times and self.tts_times:
                total_components = avg_whisper + avg_llm + avg_tts
                print(f"\n   📊 Desglose:")
                print(f"      • Whisper: {(avg_whisper/total_components)*100:.1f}%")
                print(f"      • LLM: {(avg_llm/total_components)*100:.1f}%")
                print(f"      • TTS: {(avg_tts/total_components)*100:.1f}%")
        
        # === ESTADÍSTICAS GLOBALES ===
        print(f"\n📊 ESTADÍSTICAS GLOBALES:")
        print(f"   • Total de sesiones: {self.total_sessions}")
        print(f"   • Total de mensajes: {self.total_messages_ever}")
        if self.total_sessions > 0:
            avg_msgs = self.total_messages_ever / self.total_sessions
            print(f"   • Promedio de mensajes por sesión: {avg_msgs:.1f}")
        
        print("\n" + "="*70)
        print(f"⏰ Última actualización: {datetime.datetime.now().strftime('%H:%M:%S')}")
        print("="*70 + "\n")
    
    def _get_state_emoji(self):
        """Retorna emoji según el estado"""
        emojis = {
            "IDLE": "💤",
            "WAKE_DETECTED": "👂",
            "LISTENING": "🎤",
            "PROCESSING_LLM": "🧠",
            "SPEAKING": "🔊"
        }
        return emojis.get(self.current_state, "❓")

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n🔴 Dashboard detenido por el usuario")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()