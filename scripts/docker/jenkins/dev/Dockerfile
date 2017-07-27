# Own Ubuntu base
FROM jderobot/jderobot:dev

LABEL manteiner Aitor Martínez Fernández+aitor.martinez.fernandez@gmail.com


# install basic packages
RUN apt update && apt install -q -y \
    whois \
    openssh-server \
    openjdk-8-jdk \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /var/run/sshd

RUN useradd -p `mkpasswd jenkins` -d /home/jenkins -m -g users -s /bin/bash jenkins

RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/jenkins/.bashrc

RUN /usr/sbin/sshd

COPY ./compile.sh /bin/




