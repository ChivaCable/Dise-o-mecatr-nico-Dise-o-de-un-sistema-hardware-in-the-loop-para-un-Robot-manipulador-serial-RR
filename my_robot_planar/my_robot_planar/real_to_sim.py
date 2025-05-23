#!/usr/bin/env python3
# real_to_sim.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class RealToSimBridge(Node):
    def __init__(self):
        super().__init__('real_to_sim_bridge')

        self.subscription = self.create_subscription(
            Point,
            '/joint_state',
            self.callback_estado,
            10)

        self.pub_motor1 = self.create_publisher(Float64, '/topic_1', 10)
        self.pub_motor2 = self.create_publisher(Float64, '/topic_2', 10)

    def callback_estado(self, msg):
        val1 = Float64()
        val1.data = -msg.x  # Invertir el signo
        self.pub_motor1.publish(val1)

        val2 = Float64()
        val2.data = -msg.y  # Invertir el signo
        self.pub_motor2.publish(val2)

        self.get_logger().info(
            f"Gazebo ← x: {val1.data:.3f}, y: {val2.data:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = RealToSimBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
