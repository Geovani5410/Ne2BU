import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import datetime
import os

class HistoryNode(Node):
    def __init__(self):
        super().__init__('history_node')
        
        self.conversation_history = []
        self.history_dir = 'conversation_history'
        self.session_active = False
        self.save_timer = None
        
        os.makedirs(self.history_dir, exist_ok=True)
        self.get_logger().info(f'Nodo de historial iniciado. Archivos en: {os.path.abspath(self.history_dir)}')
        
        # Suscriptores
        self.user_sub = self.create_subscription(
            String,
            'transcripcion_voz',
            self.user_text_callback,
            10
        )
        
        self.llm_sub = self.create_subscription(
            String,
            'llm_response_text',
            self.llm_response_callback,
            10
        )
        
        self.activation_state_sub = self.create_subscription(
            Bool,
            'mic_activation_state',
            self.activation_state_callback,
            10
        )
    
    def user_text_callback(self, msg):
        """Guarda la entrada del usuario"""
        user_input = msg.data.strip()
        if user_input:
            entry = {
                "role": "user",
                "text": user_input,
                "timestamp": datetime.datetime.now().isoformat()
            }
            self.conversation_history.append(entry)
            self.get_logger().info(f'[HISTORIAL] Usuario: "{user_input[:50]}..."')
    
    def llm_response_callback(self, msg):
        """Guarda la respuesta del LLM"""
        llm_output = msg.data.strip()
        if llm_output:
            entry = {
                "role": "nebu",
                "text": llm_output,
                "timestamp": datetime.datetime.now().isoformat()
            }
            self.conversation_history.append(entry)
            self.get_logger().info(f'[HISTORIAL] Nebu: "{llm_output[:50]}..."')
    
    def activation_state_callback(self, msg):
        """Maneja el inicio y fin de sesiones"""
        is_active = msg.data
        
        if is_active is True:
            # Nueva sesión iniciada
            if not self.session_active:
                self.get_logger().info('[HISTORIAL] Nueva sesión iniciada')
                self.session_active = True
                # NO limpiar aquí, solo marcar que hay sesión activa
        
        elif is_active is False:
            # Sesión terminada
            if self.session_active:
                self.get_logger().info('[HISTORIAL] Sesión terminada, programando guardado...')
                self.session_active = False
                
                # Cancelar timer anterior si existe
                if self.save_timer is not None:
                    self.save_timer.cancel()
                
                # Esperar 1 segundo para asegurar que llegó todo
                self.save_timer = self.create_timer(1.0, self.save_and_reset)
    
    def save_and_reset(self):
        """Guarda el historial y limpia para la próxima sesión"""
        # Cancelar el timer para que no se repita
        if self.save_timer is not None:
            self.save_timer.cancel()
            self.save_timer = None
        
        # Validar que hay contenido para guardar
        if len(self.conversation_history) < 2:
            self.get_logger().warn(f'[HISTORIAL] Muy poco contenido para guardar ({len(self.conversation_history)} mensajes)')
            self.conversation_history = []
            return
        
        # Verificar que hay al menos un mensaje de cada rol
        roles = [msg['role'] for msg in self.conversation_history]
        if 'user' not in roles or 'nebu' not in roles:
            self.get_logger().warn('[HISTORIAL] Conversación incompleta (falta user o nebu)')
            self.conversation_history = []
            return
        
        self.save_history_to_json()
    
    def save_history_to_json(self):
        """Guarda el historial en archivo JSON"""
        if not self.conversation_history:
            self.get_logger().warn('[HISTORIAL] No hay datos para guardar')
            return
        
        now = datetime.datetime.now()
        filename = now.strftime("%Y%m%d_%H%M%S") + ".json"
        full_path = os.path.join(self.history_dir, filename)
        
        # Crear estructura completa con metadata
        history_data = {
            "session_start": self.conversation_history[0]["timestamp"],
            "session_end": self.conversation_history[-1]["timestamp"],
            "total_messages": len(self.conversation_history),
            "conversation": self.conversation_history
        }
        
        try:
            with open(full_path, 'w', encoding='utf-8') as f:
                json.dump(history_data, f, ensure_ascii=False, indent=4)
            
            self.get_logger().info(f'[HISTORIAL] ✓ Guardado exitoso: {filename} ({len(self.conversation_history)} mensajes)')
            
        except Exception as e:
            self.get_logger().error(f'[HISTORIAL] ✗ Error al guardar: {e}')
        
        # Limpiar historial después de guardar
        self.conversation_history = []

def main(args=None):
    rclpy.init(args=args)
    history_node = HistoryNode()
    
    try:
        rclpy.spin(history_node)
    except KeyboardInterrupt:
        # Si se interrumpe, intentar guardar lo que haya
        if history_node.conversation_history:
            history_node.get_logger().info('[HISTORIAL] Guardando por interrupción...')
            history_node.save_history_to_json()
    finally:
        history_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()