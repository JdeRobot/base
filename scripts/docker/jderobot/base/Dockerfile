FROM jderobot/gzserver

MAINTAINER Aitor Martinez+aitor.martinez.fernandez@gmail.com

LABEL Usage.run="docker run -itP --rm jderobot/jderobot:base world [world_name]"
LABEL Usage.listWorlds="docker run --rm jderobot/jderobot:base lsworld"
LABEL Usage.enter="docker run -itP --rm jderobot/jderobot:base"


RUN sh -c 'echo deb http://zeroc.com/download/apt/ubuntu16.04 stable main > /etc/apt/sources.list.d/zeroc.list'
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv 5E6DA83306132997

RUN sh -c 'echo "deb http://jderobot.org/apt xenial main" > /etc/apt/sources.list.d/jderobot.list'

RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys B0E7F58E82C8091DF945A0453DA08892EE69A25C


# JdeRobot
RUN apt-get update && apt-get -y  install \
	jderobot \
	xvfb \
	&& rm -rf /var/lib/apt/lists/*

# setup environment JdeRobot
EXPOSE 8990
EXPOSE 8991
EXPOSE 8992
EXPOSE 8993
EXPOSE 8994
EXPOSE 8995
EXPOSE 8996
EXPOSE 8997
EXPOSE 8998
EXPOSE 8999
EXPOSE 9000
EXPOSE 9800
EXPOSE 9900
EXPOSE 9989
EXPOSE 9990
EXPOSE 9991
EXPOSE 9992
EXPOSE 9993
EXPOSE 9994
EXPOSE 9995
EXPOSE 9996
EXPOSE 9997
EXPOSE 9998
EXPOSE 9999

COPY ./base_entrypoint.sh /

ENTRYPOINT ["/base_entrypoint.sh"]

CMD ["bash"]

