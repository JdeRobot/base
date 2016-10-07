FROM jderobot/jderobot:base

MAINTAINER Aitor Martinez+aitor.martinez.fernandez@gmail.com

LABEL Usage.run="docker run -itP --rm -p 7681:7681 jderobot/jderobot:gzweb world [world_name]"
LABEL Usage.listWorlds="docker run --rm jderobot/jderobot:gzweb lsworld"
LABEL Usage.enter="docker run --rm jderobot/jderobot:gzweb"

# install packages gzweb dependences
RUN apt-get update && apt-get install -q -y \
    build-essential \
    cmake \
    imagemagick \
    libboost-all-dev \
    libgts-dev \
    libjansson-dev \
    libtinyxml-dev \
    mercurial \
    nodejs \
    nodejs-legacy \
    npm \
    pkg-config \
    psmisc \
    && rm -rf /var/lib/apt/lists/*

# clone gzweb
RUN hg clone https://bitbucket.org/aitormf/gzweb ~/gzweb

# build gzweb
COPY ./installGzweb.sh /
RUN /installGzweb.sh


# setup environment GzWeb
EXPOSE 8080
EXPOSE 7681

COPY ./gzweb_entrypoint.sh /

ENTRYPOINT ["/gzweb_entrypoint.sh"]

CMD ["bash"]

