#!/bin/bash
set -e

source /opt/ros/kinetic/setup.bash

if [ "$1" = 'video' ]; then
   
    cd /cfg
    cameraserver $2
    
elif [ "$1" = 'lsvideos' ]; then

    ls /cfg

else
    exec "$@"
fi
