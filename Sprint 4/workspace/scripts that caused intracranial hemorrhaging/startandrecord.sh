ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /opt/ros/humble/share/usb_cam/config/params_1.yaml &
sleep 3 & echo "Waiting 3 seconds..."
ros2 bag record /image_raw/compressed
