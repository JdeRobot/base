#!/bin/bash

solodir=/tmp/solo
if [ -d $solodir ]; then
   rm -rf $solodir
fi

cp -r /opt/jderobot/share/jderobot/drivers/solodrone $solodir

wd=$(pwd)

cd $solodir

./px4 . ./posix-configs/SITL/init/ekf2/iris
