#!/bin/bash

if [ ! -f /etc/apt/sources.list.d/ros-latest.list ]; then
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
	sudo apt-get update
fi

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
cd ..
sudo rm -rf rqt_ez_publisher
popd
