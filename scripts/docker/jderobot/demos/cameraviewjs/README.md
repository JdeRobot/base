JdeRobot CameraViewJS Demo Image
========

Demo of CameraViewJS. This image depends on  [JdeRobot Official Image](https://hub.docker.com/r/jderobot/jderobot/)

# Usage
* Download and run: 
```sh
docker run -d --name demo_cameraviewjs --device="/dev/video0:/dev/video0:rw" -p 7777:7777 -p 9999:9999 -p 11000:11000 jderobot/demos:cameraviewjs
```

To see cameraviewjs, put in your browser: "localhost:7777"
