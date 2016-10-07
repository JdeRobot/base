FROM gazebo:libgazebo7

MAINTAINER Aitor Martinez+aitor.martinez.fernandez@gmail.com

LABEL Usage.run="docker run -itP --rm jderobot/jderobot:base world [world_name]"
LABEL Usage.listWorlds="docker run --rm jderobot/jderobot:base lsworld"
LABEL Usage.enter="docker run -itP --rm jderobot/jderobot:base"

#RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
#RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

RUN sh -c 'echo deb http://ppa.launchpad.net/v-launchpad-jochen-sprickerhof-de/pcl/ubuntu trusty main > /etc/apt/sources.list.d/v-launchpad-jochen-sprickerhof-de-pcl-trusty.list'
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D51148574AAB18C259B43628D8A3751519274DEF

RUN sh -c 'echo deb http://zeroc.com/download/apt/ubuntu14.04 stable main > /etc/apt/sources.list.d/zeroc.list'
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv 5E6DA83306132997

RUN apt-get update && apt-get -y  install \
	build-essential \ 
	libtinyxml-dev \ 
	libtbb-dev \
	libxml2-dev \
	libqt4-dev \
	pkg-config \ 
	libprotoc-dev \ 
	libfreeimage-dev \ 
	libprotobuf-dev \ 
	protobuf-compiler \ 
	libboost-all-dev \ 
	freeglut3-dev \ 
	cmake \ 
	libtar-dev \ 
	libcurl4-openssl-dev \ 
	libcegui-mk2-dev \ 
	libswscale-dev \ 
	libavformat-dev \ 
	libavcodec-dev \ 
	git \
	&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y install \ 
	libxml++2.6-2 \ 
	libxml++2.6-dev \ 
	libvtk5.8 \ 
	libvtk5-dev \
	&& rm -rf /var/lib/apt/lists/*

# gtk libraries
RUN apt-get update && apt-get -y install \ 
	libgtk2.0-0 \ 
	libgtk2.0-bin \ 
	libgtk2.0-cil \ 
	libgtk2.0-common \ 
	libgtk2.0-dev \ 
	libgtkgl2.0-1 \
	&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y install \ 
	libgtkgl2.0-dev \ 
	libgtkglext1 \ 
	libgtkglext1-dev \ 
	libglademm-2.4-dev \ 
	libgtkmm-2.4-dev \
	&& rm -rf /var/lib/apt/lists/*

	
RUN apt-get update && apt-get -y install \
	libgnomecanvas2-0 \ 
	libgnomecanvas2-dev \ 
	libgnomecanvasmm-2.6-dev \
	&& rm -rf /var/lib/apt/lists/*
	
RUN apt-get update && apt-get -y install \ 
	libgnomecanvasmm-2.6-1c2a \ 
	libgtkglextmm-x11-1.2-0 \ 
	libgtkglextmm-x11-1.2-dev \
	&& rm -rf /var/lib/apt/lists/*

# GSL libraries
RUN apt-get update && apt-get -y install \ 
	libgsl0ldbl \ 
	libgsl0-dev \
	&& rm -rf /var/lib/apt/lists/*

# PCL
RUN apt-get update \
	&& apt-get -y install libpcl-all \
	&& rm -rf /var/lib/apt/lists/*

# Gazebo7
RUN apt-get update && apt-get -y install \ 
#	gazebo7 \ 
#	libgazebo7-dev \ 
	libopencv-dev \
	xvfb \
	&& rm -rf /var/lib/apt/lists/*


# install ICE
RUN apt-get update && apt-get -y install \ 
	libzeroc-ice3.6 \ 
	zeroc-ice-utils \ 
	libzeroc-icestorm3.6 \ 
	zeroc-ice-slice \ 
	libzeroc-ice-dev \
	&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y install libbz2-dev python-pip && rm -rf /var/lib/apt/lists/*


RUN pip install zeroc-ice

# JdeRobot

COPY ./installJderobot.sh /

RUN ./installJderobot.sh

#RUN sh -c 'mv /usr/local/jderobot/* /usr/local/lib/jderobot'

RUN rm -rf /var/lib/apt/lists/*
RUN rm -rf /git

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

