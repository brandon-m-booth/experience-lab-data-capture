#!/bin/bash

# ROS dependencies
sudo apt-get install ros-indigo-desktop-full -y
sudo apt-get install ros-indigo-audio-common -y
sudo apt-get install ros-indigo-freenect-stack -y

# Audio dependencies
sudo apt-get install libmp3lame-dev

# RQT dependencies
pushd /opt/ros/indigo/share
sudo git clone https://github.com/OTL/rqt_ez_publisher.git
cd rqt_ez_publisher
sudo python setup.py install
rm ~/.config/ros.org/rqt_gui.ini
popd
