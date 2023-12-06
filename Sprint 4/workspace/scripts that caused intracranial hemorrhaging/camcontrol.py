#!/usr/bin/env python3
import sys
import os
import subprocess
import signal
import rclpy

from rclpy.node import Node
from rclpy.logging import get_logger

class CamControllerNode(Node):
    def __init__(self):
        super().__init__('cam_controller')
        self.start_script = "startcam.sh"
        self.lock_file = ".startcam_lock"
        self.record_file = "recorded_data.bag"
        self.record_process = None

    def check_startcam_running(self):
        if os.path.exists(self.lock_file):
            with open(self.lock_file, 'r') as lock_file:
                check_pid = lock_file.read().strip()
                if check_pid and subprocess.call(["ps", "-p", check_pid]) == 0:
                    return True
        return False

    def start_startcam(self):
        if self.check_startcam_running():
            self.get_logger().error("Another instance of %s is already running.", self.start_script)
            return

        try:
            subprocess.Popen([f"./{self.start_script}"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.get_logger().info("Started %s", self.start_script)
        except Exception as e:
            self.get_logger().error(f"Failed to start {self.start_script}: {e}")
            
    def start_recordcamcompressed(self):
        if not self.check_startcam_running():
            self.get_logger().error("%s is not running. Cannot start recording.", self.start_script)
            return

        if self.record_process:
            self.get_logger().error("Recording is already in progress.")
            return

        self.record_process = subprocess.Popen(["ros2", "bag", "record", "/image_raw/compressed"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.get_logger().info("Started recording (PID: %d)", self.record_process.pid)

    def stop_recordcamcompressed(self):
        if not self.record_process:
            self.get_logger().warning("No recording is currently active.")
            return

        try:
            self.record_process.send_signal(signal.SIGINT)
            self.record_process.wait()
            self.record_process = None
            self.get_logger().info("Stopped recording")
        except Exception as e:
            self.get_logger().error(f"Failed to stop recording: {e}")

    def stop_startcam(self):
        if not os.path.exists(self.lock_file):
            self.get_logger().warning("%s is not running.", self.start_script)
            return

        try:
            with open(self.lock_file, 'r') as lock_file:
                pid = lock_file.read().strip()
                if pid:
                    subprocess.call(["pkill", "-f", f"usb_cam"])
                    self.get_logger().info("Stopped %s (PID: %s)", self.start_script, pid)
                    os.remove(self.lock_file)
                else:
                    self.get_logger().error("Error: Invalid PID found in the lock file.")
        except Exception as e:
            self.get_logger().error(f"Failed to stop {self.start_script}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CamControllerNode()

    while True:
        choice = input("Choose an option:\n1. Start startcam.sh\n2. Stop startcam.sh\n3. Start Recording\n4. Stop Recording\n5. Quit\nEnter your choice: ")

        if choice == '1':
            node.start_startcam()
        elif choice == '2':
            node.stop_startcam()
        elif choice == '3':
            node.start_recordcamcompressed()
        elif choice == '4':
            node.stop_recordcamcompressed()
        elif choice == '5':
            node.destroy_node()
            rclpy.shutdown()
            print("Goodbye!")
            sys.exit(0)
        else:
            print("Invalid choice")

if __name__ == '__main__':
    main()

