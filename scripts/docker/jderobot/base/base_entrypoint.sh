#!/bin/bash
set -e

source /usr/local/share/jderobot/gazebo/gazebo-setup.sh

if [ "$1" = 'world' ]; then

    Xvfb -shmem -screen 0 1280x1024x24 &
   
    DISPLAY=:0 gzserver $2
    
elif [ "$1" = 'lsworld' ]; then

    ls /usr/local/share/jderobot/gazebo/worlds

else
    exec "$@"
fi


