#Demo CameraViewJS
FROM jderobot/jderobot

LABEL manteiner Aitor Martínez Fernández+aitor.martinez.fernandez@gmail.com
LABEL Usage.run="docker run -d --name demo_cameraviewjs --device="/dev/video0:/dev/video0:rw" -p 7777:7777 -p 9999:9999 -p 11000:11000 jderobot/demos:cameraviewjs"

RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc 

COPY ./democameraviewjs.sh /usr/bin
CMD ["democameraviewjs.sh"]
