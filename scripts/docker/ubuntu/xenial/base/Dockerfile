# Own Ubuntu base
FROM ubuntu:xenial

LABEL manteiner Aitor Martínez Fernández+aitor-martinez.fernandez@gmail.com


# install basic packages
RUN apt update && apt install -q -y \
    wget \
    sudo \
    nano \
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# install graphics packages
RUN apt update && apt install -q -y \
    binutils \
    mesa-utils \
    module-init-tools \
    x-window-system\
    && rm -rf /var/lib/apt/lists/*

# install sublime
RUN wget https://download.sublimetext.com/sublime-text_build-3126_amd64.deb \
    && dpkg -i sublime-text_build-3126_amd64.deb \
    && rm sublime-text_build-3126_amd64.deb

# Enabling bash-completion
COPY ./completion /tmp/

RUN cat /tmp/completion >> /etc/bash.bashrc


