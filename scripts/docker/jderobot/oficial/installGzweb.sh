#!/bin/bash

cd ~/gzweb
hg up jderobot2
source /usr/local/share/jderobot/gazebo/gazebo-setup.sh
source /opt/ros/kinetic/setup.bash
Xvfb -shmem -screen 0 1280x1024x24 &
export DISPLAY=:0
./deploy.sh -m local
