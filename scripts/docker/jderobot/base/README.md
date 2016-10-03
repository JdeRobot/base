JdeRobot Base
========

Docker image with gazebo 7, Ice 3.6 and all gazebo plugins of JdeRobot installed

# Use

There are Three ways to use it:

* run without arguments: open a bash to access to docker.

```sh
$ docker run -tiP --rm jderobot/jderobot:base 
```

* See all posibles worlds:

```sh
$ docker run -tiP --rm jderobot/jderobot:base lsworld 
```

* Run a world: 

```sh
$ docker run -tiP --rm jderobot/jderobot:base world world_name 
```
