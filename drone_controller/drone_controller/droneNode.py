#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class droneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.sub = self.create_subscription(
            String, 
            '/coordinator/commands', 
            self.listener_callback, 
            10)
        self.get_logger().info('Basics-only Drone Node started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = droneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Drone Node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()