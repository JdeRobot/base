#!/bin/bash
set -e

if [ "$1" = 'video' ]; then
   
    cd /cfg
    cameraserver $2
    
elif [ "$1" = 'lsvideos' ]; then

    ls /cfg

else
    exec "$@"
fi
