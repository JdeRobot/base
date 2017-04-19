Docker
========
This directory contains all Dockerfiles of JdeRobot official Docker images.

# Run and download Docker Image

```sh
$ docker run -tiP --rm jderobot/jderobot:tag_name 
```

For more information go to [Teaching Robotics page](http://jderobot.org/Teaching_robotics_with_JdeRobot#Run_Exercises)

# Build Image

Go to Dockerfiles directory, for example jderobot/base and run:

```sh
$ docker build --tag="jderobot:base" . 
```

# Edit Dockerfile

Follow [these instructions](https://docs.docker.com/engine/reference/builder/)
