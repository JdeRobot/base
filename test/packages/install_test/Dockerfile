FROM ubuntu:xenial


LABEL manteiner Aitor Martínez Fernández+aitor.martinez.fernandez@gmail.com

RUN apt-get update && apt-get -y  install \
    wget \
    && rm -rf /var/lib/apt/lists/*


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
RUN sh -c 'echo "deb http://jderobot.org/aptest xenial main" > /etc/apt/sources.list.d/jderobot.list'
RUN wget -qO - www.jderobot.org/aptest/aptest.key | apt-key add -

########## Install JdeRobot ##############
RUN apt-get update && apt-get -y  install\
    jderobot \
    && rm -rf /var/lib/apt/lists/*


########## Install JdeRobot ##############
RUN apt-get update && apt-get -y  purge\
    jderobot \
    && rm -rf /var/lib/apt/lists/*
