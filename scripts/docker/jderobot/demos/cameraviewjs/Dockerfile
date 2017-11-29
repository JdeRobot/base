#Demo CameraViewJS
FROM ubuntu:xenial

LABEL manteiner Aitor Martínez Fernández+aitor.martinez.fernandez@gmail.com
LABEL Usage.run="docker run -d --name demo_cameraviewjs --device="/dev/video0:/dev/video0:rw" -p 7777:7777 -p 9999:9999 -p 11000:11000 jderobot/demos:cameraviewjs"

########## setup Repositories ##############

## ROS ##
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

## ZeroC ##
RUN sh -c 'echo deb http://zeroc.com/download/apt/ubuntu16.04 stable main > /etc/apt/sources.list.d/zeroc.list'
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv 5E6DA83306132997

## Gazebo ##
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key 67170598AF249743


## JdeRobot ##
RUN sh -c 'echo "deb http://jderobot.org/apt xenial main" > /etc/apt/sources.list.d/jderobot.list'
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv B0E7F58E82C8091DF945A0453DA08892EE69A25C

########## Install Tools ##############
RUN apt-get update && apt-get -y  install \
	jderobot-cameraserver \
	jderobot-webtools \
	&& rm -rf /var/lib/apt/lists/*



COPY ./democameraviewjs.sh /usr/bin
CMD ["democameraviewjs.sh"]
