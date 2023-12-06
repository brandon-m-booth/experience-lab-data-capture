#!/bin/bash
ros2 run usb_cam usb_cam_node_exe & ros2 bag record /image_raw/compressed
