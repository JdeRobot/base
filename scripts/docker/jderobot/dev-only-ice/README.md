JdeRobot Image for development, only ICE 
========

JdeRobot Image for development that includes all JdeRobot dependencies without ROS. This image depends on  [JdeRobot Ubuntu Image](https://hub.docker.com/r/jderobot/ubuntu/)

It contains:
* JdeRobot dependencies without ROS packages
* wget, sudo, bash-completion packages (from Ubuntu)
* binutils, mesa-utils, module-init-tools, x-window-system graphics packages (from Ubuntu)
* Nano and sublime TextEditors installed (from Ubuntu) ... 

# Usage
* Download: 
```sh
docker pull jderobot/jderobot:dev-only-ice
```
* run without GUI: 
```sh
docker run -ti jderobot/jderobot:dev-only-ice bash
```
* run with GUI: 
```sh
xhost +local:root
docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" jderobot/jderobot:dev-only-ice bash
```
