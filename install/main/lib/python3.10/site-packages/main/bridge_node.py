import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String # Se añade String para escuchar la respuesta del LLM

class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')

        # 1. Publicador hacia el topic del micro-ROS (ESP32)
        self.led_publisher = self.create_publisher(Bool, '/micro_ros_led', 10)
        
        # 2. Suscripción al estado del micrófono (mic_activation_state)
        # Esto viene del WakeNode o LLMNode y significa: "Whisper DEBE grabar ahora"
        self.mic_subscription = self.create_subscription(
            Bool,
            'mic_activation_state',
            self.mic_state_callback, 
            10
        ) 
        
        # 3. NUEVA SUSCRIPCIÓN: Escucha la respuesta del LLM (llm_response_text)
        # Esto significa: "El sistema va a hablar, APAGA el indicador de micro"
        self.tts_start_subscription = self.create_subscription(
            String,
            'llm_response_text',
            self.tts_start_callback,
            10
        )

        self.get_logger().info('BridgeNode listo. Controlando LED con estado de micrófono y TTS.')

    def publish_led_state(self, state: bool):
        """Función auxiliar para publicar el estado del LED."""
        msg = Bool()
        msg.data = state
        self.led_publisher.publish(msg)
        
        state_str = "ENCENDIDO (Whisper Activo/Listo)" if state else "APAGADO (TTS Hablando/VOSK Esperando)"
        self.get_logger().info(f"-> Publicando /micro_ros_led: {state_str}")

    def mic_state_callback(self, msg: Bool):
        """
        Callback al recibir un mensaje en 'mic_activation_state'.
        Esto es la señal para encender el LED (micrófono activo para la voz del usuario).
        """
        if msg.data is True:
            # Micrófono activo (Whisper debe grabar, o LLM pide nueva entrada) -> LED ON
            self.publish_led_state(True)
        else:
             # Micrófono inactivo (sistema volviendo al estado de VOSK) -> LED OFF
            self.publish_led_state(False)

    def tts_start_callback(self, msg: String):
        """
        Callback al recibir un mensaje en 'llm_response_text'.
        Esto es la señal para apagar el LED (el sistema va a responder/TTS va a hablar).
        """
        if msg.data.strip():
            # Texto recibido significa que el TTS va a empezar
            # Apagamos el LED, el micro del usuario no es necesario en este momento
            self.get_logger().info("Recibida respuesta del LLM. Apagando LED para indicar respuesta.")
            self.publish_led_state(False)
        # NOTA: El LED se volverá a encender por el LLMNode/WakeNode a través de mic_activation_state
        # cuando el TTS termine de hablar (a través de dialogue_continue).

def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()