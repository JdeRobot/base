JdeRobot Image for development, with ROS
========

JdeRobot Image for development that includes all JdeRobot dependencies. This image depends on  [JdeRobot Dev-only-ICE](https://hub.docker.com/r/jderobot/jderobot/tags/)

It contains:
* JdeRobot dependencies
* wget, sudo, bash-completion packages (from Ubuntu)
* binutils, mesa-utils, module-init-tools, x-window-system graphics packages (from Ubuntu)
* Nano and sublime TextEditors installed (from Ubuntu) ... 

# Usage
* Download: 
```sh
docker pull jderobot/jderobot:dev
```
* run without GUI: 
```sh
docker run -ti jderobot/jderobot:dev bash
```
* run with GUI: 
```sh
xhost +local:root
docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" jderobot/jderobot:dev bash
```
