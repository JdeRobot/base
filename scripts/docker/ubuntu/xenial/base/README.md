JdeRobot Ubuntu Image
========

This is a image of Ubuntu with basic packages, can be used to try JdeRobot package installation. This image depends on  [Ubuntu official image](https://hub.docker.com/_/ubuntu/)

It contains:
* wget, sudo, bash-completion packages
* binutils, mesa-utils, module-init-tools, x-window-system graphics packages
* Nano and sublime TextEditors installed ...

# Usage
* Download: 
```sh
docker pull jderobot/ubuntu:base
```
* run without GUI: 
```sh
docker run -ti jderobot/ubuntu:base bash
```
* run with GUI: 
```sh
xhost +local:root
docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" jderobot/ubuntu:base bash
```

