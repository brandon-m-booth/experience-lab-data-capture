#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 bag record /image_raw/compressed > logfile.txt 2>&1
