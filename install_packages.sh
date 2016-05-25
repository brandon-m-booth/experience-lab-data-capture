#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Genera packages
sudo apt-get install libtool
sudo apt-get install pkg-config
sudo apt-get install build-essential
sudo apt-get install autoconf
sudo apt-get install automake
sudo apt-get install uuid-dev

# Add ROS package repository
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
#sudo git clone https://github.com/OTL/rqt_ez_publisher.git
sudo git clone https://github.com/brandon-m-booth/rqt_ez_publisher.git
cd rqt_ez_publisher
sudo git checkout indigo-devel-annotators
sudo python setup.py install
rm ~/.config/ros.org/rqt_gui.ini
popd

# ZMQ
# (first install libsodium)
pushd /tmp
git clone git://github.com/jedisct1/libsodium.git
cd libsodium
./autogen.sh
./configure
make check
sudo make install
sudo ldconfig
cd ..
# (then install ZMQ)
tar -xvzf $DIR/install_packages/zeromq-4.1.4.tar.gz
cd zeromq-4.1.4
./configure
make
sudo make install
sudo ldconfig
popd
