#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 bag record /image_raw > rawlog.txt 2>&1
