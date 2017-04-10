#!/bin/bash

cameraserver cameraserver.cfg &

wd=$(pwd)
cd /usr/local/share/jderobot/webtools/cameraviewjs
nodejs run.js
cd $wd