#!/bin/bash

cd ~/gzweb
hg up jderobot
source /usr/local/share/jderobot/gazebo/gazebo-setup.sh
Xvfb -shmem -screen 0 1280x1024x24 &
export DISPLAY=:0
./deploy.sh -m local
