FROM jderobot/jderobot


LABEL manteiner Aitor Martínez Fernández+aitor.martinez.fernandez@gmail.com

RUN apt-get update && apt-get -y  install \
    wget \
    && rm -rf /var/lib/apt/lists/*

########## setup Repositories ##############
## JdeRobot Test##
RUN sh -c 'echo "deb http://jderobot.org/aptest xenial main" > /etc/apt/sources.list.d/jderobot.list'
RUN wget -qO - www.jderobot.org/aptest/aptest.key | apt-key add -

########## Upgrade JdeRobot ##############
RUN apt-get update && apt-get -y  install --only-upgrade \
    jderobot \
    && rm -rf /var/lib/apt/lists/*




