#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class KeystrokeListenerNode(Node):
    def __init__(self):
        super().__init__('keystroke_listener')
        self.publisher_ = self.create_publisher(String, 'keystrokes_topic', 10)
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press)
        self.keyboard_listener.start()

    def on_key_press(self, key):
        try:
            key_char = key.char
        except AttributeError:
            key_char = str(key)

        keystroke_msg = String()
        keystroke_msg.data = f'Key pressed: {key_char}'
        self.publisher_.publish(keystroke_msg)
        print(f"Published message: {keystroke_msg}") 

def main(args=None):
    rclpy.init(args=args)
    node = KeystrokeListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.keyboard_listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
