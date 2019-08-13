#!/usr/bin/env sh
#mkdir turtlebot_ws/src && cd turtlebot_ws/src
mkdir src && cd src
catkin_init_workspace

git clone https://github.com/yujinrobot/kobuki_msgs.git
git clone https://github.com/yujinrobot/kobuki_desktop.git
git clone https://github.com/yujinrobot/kobuki.git
mv kobuki_desktop/kobuki_gazebo kobuki_desktop/kobuki_gazebo_plugins ./
mv kobuki/kobuki_description kobuki/kobuki_safety_controller \
   kobuki/kobuki_random_walker ./
rm -rf kobuki kobuki_desktop


git clone https://github.com/yujinrobot/yujin_ocs.git
mv yujin_ocs/yocs_cmd_vel_mux yujin_ocs/yocs_controllers .
rm -rf yujin_ocs

sudo apt-get install ros-melodic-kobuki-* -y
sudo apt-get install ros-melodic-ecl-streams -y

catkin_make
source devel/setup.bash
#or
#sudo cp devel/share/* /opt/ros/melodic/share/
#source /opt/ros/melodic/setup.bash
