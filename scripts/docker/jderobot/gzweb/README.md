JdeRobot GzWeb
========

Docker image with gazebo 7, Ice 3.6, gzweb as gazebo GUI and all gazebo plugins of JdeRobot installed

# Use

There are Three ways to use it:

* run without arguments: open a bash to access to docker.

```sh
$ docker run -tiP --rm jderobot/jderobot:gzweb 
```

* See all posibles worlds:

```sh
$ docker run -tiP --rm jderobot/jderobot:gzweb lsworld 
```

* Run a world: 

```sh
$ docker run -tiP --rm -p 7681:7681 jderobot/jderobot:gzweb world world_name 
```

[![Introrob_py + GzWeb in Windows](https://img.youtube.com/vi/YoJYahFtEyg/0.jpg)](https://www.youtube.com/watch?v=YoJYahFtEyg)
