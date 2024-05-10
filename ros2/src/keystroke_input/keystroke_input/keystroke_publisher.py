#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeystrokePublisher(Node):

    def __init__(self):
        super().__init__('keystroke_publisher') 
        self.subscription = self.create_subscription(
            String,
            'keystrokes_topic',  # Change this topic to match the keystrokes topic name
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    keystroke_publisher = KeystrokePublisher()
    rclpy.spin(keystroke_publisher)
    keystroke_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
