#!/bin/bash

cameraserver cameraserver.cfg &

wd=$(pwd)
cd /opt/jderobot/share/jderobot/webtools/cameraviewjs
nodejs run.js
cd $wd