import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')

        # Suscribirse al topic que mandan tus otros nodos
        self.subscription = self.create_subscription(
            Bool,
            '/led_cmd',     # topic global
            self.listener_callback,
            10)
        
        # Publicar hacia el topic del micro-ROS (ESP32)
        self.publisher = self.create_publisher(
            Bool,
            '/micro_ros_led',
            10)
        
        self.get_logger().info('BridgeNode listo.')

    def listener_callback(self, msg):
        self.get_logger().info(f"Recibido /led_cmd: {msg.data}")
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#char ssid[] = "INTERNET ZONA _CERO";        
#char agent_ip[] = "192.168.1.100"; 
#define AGENT_PORT 8888