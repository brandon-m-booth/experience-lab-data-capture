#!/bin/bash

# Cleanup and exit func
cleanup() {
	echo "Received Ctrl+C, terminating the script"
	# Terminate background processes
	pkill -P $$
	echo "Press Enter to exit..."
	read
	exit 1
}

# Trap Ctrl+C and call cleanup function
trap cleanup INT

# ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /opt/ros/humble/share/usb_cam/config/params_1.yaml & echo "Waiting 10 seconds..."
./startcam.sh & echo "Waiting 10 seconds..."

sleep 10

echo "Starting camera bag recording"

# ros2 bag record /image_raw/compressed & echo "Camera node started + bag recording.  CTRL+C to terminate everything"
./recordcamcompressed.sh & echo "Camera node started + bag recording.  Ctrl+C to terminate all"


echo "Terminating..." & cleanup
