# Instructions to install Mavros for simulate drones on Ubuntu 16.04

## Setting up the environment

**Steps**

1. Install MavLink from this [website](https://mavlink.io/en/getting_started/installation.html)
2. Install MavROS from debian packages [website](https://dev.px4.io/en/ros/mavros_installation.html)
3. Copy the solo plugins:
```
sudo cp -r ~/jderobot/src/drivers/gazebo/plugins/solo /opt/jderobot/share/jderobot/gazebo/plugins
```
**WARNING: these plugins are precompiled from an ubuntu 16.04 machine so it's only compatible with that version of Ubuntu. We are working to merge this step in the compilation chain**
4. Change the sources of JdeRobot assets:
```
sudo nano /opt/jderobot/share/jderobot/gazebo/gazebo-setup.sh
```
Add the following at the end of the list of this two environment variables `export GAZEBO_PLUGIN_PATH` and `export LD_LIBRARY_PATH`
```
:/opt/jderobot/share/jderobot/gazebo/plugins/solo
```
5. Source the plugins
```
source /opt/jderobot/share/jderobot/gazebo/gazebo-setup.sh
source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh
source /opt/jderobot/setup.bash
source /opt/ros/kinetic/setup.bash
```
6. With all this done you should be able to launch the simulated drone with mavROS

## Lauching the simulation

Launch the world with the drone in it.
```
cd /opt/jderobot/share/jderobot/gazebo/launch
roslaunch follow_road.launch
```
**WARNING:** check px4.sh is loaded, if it isn't it will throw the following error ** ERROR: cannot launch node of type [mavros/px4.sh]: can't locate node [px4.sh] in package [mavros] **  if that happens, copy the px4.sh file:
```
sudo cp ~/jderobot/src/drivers/gazebo/plugins/solo/px4.sh /opt/ros/kinetic/share/mavros
```

You should have a simulated drone in gazebo running.