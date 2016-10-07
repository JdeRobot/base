#!/bin/bash

mkdir git

cd git

git clone -b docker2 https://github.com/aitormf/JdeRobot.git

cd JdeRobot

mkdir build

cd build

cmake .. -Dbuild-default=OFF -Dbuild_gazeboserver=ON -Dbuild_car=ON -Dbuild_turtlebot=ON -Dbuild_flyingKinect2=ON -Dbuild_pioneer=ON -Dbuild_quadrotor2=ON -Dbuild_f1=ON

make

make install

cat /usr/local/share/jderobot/gazebo/gazebo-setup.sh >> ~/.bashrc
