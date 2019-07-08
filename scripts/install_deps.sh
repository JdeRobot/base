#!/bin/bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key 67170598AF249743

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv B6391CB2CFBA643D
sudo apt-add-repository "deb http://zeroc.com/download/Ice/3.7/ubuntu18.04 stable main"

sudo sh -c 'cat<<EOF>/etc/apt/sources.list.d/jderobot.list
# for ubuntu 16.04 LTS (64 bit)

deb [arch=amd64] http://jderobot.org/apt bionic main
EOF'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv 24E521A4

sudo apt update

sudo apt install wget

sudo apt install -y ros-melodic-roscpp ros-melodic-std-msgs ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-roscpp-core ros-melodic-rospy ros-melodic-nav-msgs ros-melodic-geometry-msgs ros-melodic-kobuki-msgs
sudo apt-get install -y libpcl-dev
sudo apt-get install libopencv-dev
sudo apt install -y build-essential libtool cmake g++ gcc git make

sudo apt install -y freeglut3 freeglut3-dev libgl1-mesa-dev libglu1-mesa

sudo apt-get install -y libgtk2.0-0 libgtk2.0-bin libgtk2.0-cil libgtk2.0-common libgtk2.0-dev libgtkgl2.0-1
sudo apt-get install -y libgtkgl2.0-dev libgtkglext1 libgtkglext1-dev libglademm-2.4-dev libgtkmm-2.4-dev 
sudo apt-get install -y libgnomecanvas2-0 libgnomecanvas2-dev  libgtkglext1-doc libgnomecanvasmm-2.6-dev
sudo apt-get install -y libgnomecanvasmm-2.6-1v5 libgtkglextmm-x11-1.2-0v5 libgtkglextmm-x11-1.2-dev

sudo apt-get install -y libgoocanvasmm-2.0-6 libgoocanvasmm-2.0-dev

sudo apt-get install -y libgsl23 gsl-bin libgsl-dev

sudo apt-get install -y libxml++2.6-2v5 libxml++2.6-dev libtinyxml-dev

sudo apt-get install -y libeigen3-dev

sudo apt-get install -y libdc1394-22 libdc1394-22-dev

sudo apt-get install -y libusb-1.0-0 libusb-1.0-0-dev

sudo apt-get install -y python-matplotlib python-pyqt5 python-pip python-numpy python-pyqt5.qtsvg python-pyqt5.qsci

sudo apt-get install -y qfi

sudo apt-get install -y qtbase5-dev libqt5script5 libqt5svg5-dev

sudo apt-get install -y libboost-system-dev libboost-filesystem-dev

sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

sudo apt install -y libzeroc-ice3.7 libzeroc-icestorm3.7 zeroc-ice-slice libzeroc-ice-dev

sudo apt-get install -y libssl-dev libbz2-dev
sudo pip2 install --upgrade pip
sudo pip2 install zeroc-ice==3.6.4

sudo apt-get install -y libopenni2-dev libopenni-dev

sudo apt-get install -y gazebo9 libgazebo9-dev

sudo apt-get install -y nodejs

sudo apt-get install -y ardronelib libgoogle-glog-dev

sudo apt install libyaml-cpp-dev








