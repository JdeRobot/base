# Instructions to install Mavros

## 1. Install MavLink from this [website](https://mavlink.io/en/getting_started/installation.html)
## 2. Install MavROS from debian package in this [website](https://dev.px4.io/en/ros/mavros_installation.html)
## 3. Launch the world and check the px4.sh file is loaded.
### 3.1. If the file is not load, the following error will appear in the shell: ERROR: cannot launch node of type [mavros/px4.sh]: can't locate node [px4.sh] in package [mavros]
### 3.2. Then, copy the file in /opt/ros/kinetic/share/mavros.
