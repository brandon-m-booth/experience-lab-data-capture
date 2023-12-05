#!/usr/bin/env python3

import os
import subprocess
import rclpy
from rclpy.node import Node

class ElDemoNode(Node):
    def __init__(self):
        super().__init__('el_demo')
        self.run_scripts()

    def run_scripts(self):
        # Execute your different scripts here
        script_paths = [
            'script1.py',
            'script2.py',
            # Add more script paths as needed
        ]

        for script_path in script_paths:
            full_path = os.path.join(os.getcwd(), '..', 'scripts', script_path)
            self.execute_script(full_path)

    def execute_script(self, script_path):
        self.get_logger().info(f"Executing {script_path}")
        subprocess.run(['python3', script_path])

def main(args=None):
    rclpy.init(args=args)
    node = ElDemoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
