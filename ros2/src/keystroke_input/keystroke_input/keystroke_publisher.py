#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class KeystrokePublisher(Node):

    def __init__(self):
        super().__init__('keystroke_publisher')
        self.publisher_ = self.create_publisher(String, 'keystrokes', 10)
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press)
        self.keyboard_listener.start()

    def on_key_press(self, key):
        try:
            key_char = key.char
        except AttributeError:
            key_char = str(key)

        msg = String()
        msg.data = key_char
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    keystroke_publisher = KeystrokePublisher()
    rclpy.spin(keystroke_publisher)
    keystroke_publisher.keyboard_listener.stop()
    keystroke_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
