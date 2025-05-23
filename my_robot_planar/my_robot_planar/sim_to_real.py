import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int16
import math

class SimToRealBridge(Node):
    def __init__(self):
        super().__init__('sim_to_real_bridge')

        # Suscripciones desde la simulación
        self.subscription_1 = self.create_subscription(
            Float64,
            '/topic_1',
            self.callback_q1,
            10
        )

        self.subscription_2 = self.create_subscription(
            Float64,
            '/topic_2',
            self.callback_q2,
            10
        )

        # Publicadores hacia el robot físico
        self.q1_pub = self.create_publisher(Int16, '/q1_command', 10)
        self.q2_pub = self.create_publisher(Int16, '/q2_command', 10)

        # Variables para limitar la tasa de publicación
        self.publish_interval = 2.0  # segundos
        self.last_publish_time_q1 = self.get_clock().now()
        self.last_publish_time_q2 = self.get_clock().now()

        self.get_logger().info('Bridge sim-to-real activo')

    def callback_q1(self, msg):
        now = self.get_clock().now()
        if (now - self.last_publish_time_q1).nanoseconds / 1e9 < self.publish_interval:
            return  # No publica aún

        self.last_publish_time_q1 = now

        grados = int(math.degrees(msg.data))  # rad → grados
        self.get_logger().info(f'Topic 1 → Q1 = {grados}°')
        self.q1_pub.publish(Int16(data=grados))

    def callback_q2(self, msg):
        now = self.get_clock().now()
        if (now - self.last_publish_time_q2).nanoseconds / 1e9 < self.publish_interval:
            return  # No publica aún

        self.last_publish_time_q2 = now

        grados = int(math.degrees(msg.data))  # rad → grados
        self.get_logger().info(f'Topic 2 → Q2 = {grados}°')
        self.q2_pub.publish(Int16(data=grados))

def main(args=None):
    rclpy.init(args=args)
    node = SimToRealBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
