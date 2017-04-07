# gazebo-setup fixed by varribas at 2016-04-08

# official Gazebo resources
. /usr/share/gazebo/setup.sh

# JdeRobot resources
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/local/share/jderobot/gazebo/models:/usr/local/share/jderobot/gazebo/worlds
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/local/share/jderobot/gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/local/share/jderobot/gazebo/plugins/pioneer:/usr/local/share/jderobot/gazebo/plugins/nao:/usr/local/share/jderobot/gazebo/plugins/kinect:/usr/local/share/jderobot/gazebo/plugins/quadrotor:/usr/local/share/jderobot/gazebo/plugins/turtlebot:/usr/local/share/jderobot/gazebo/plugins/flyingkinect:/usr/local/share/jderobot/gazebo/plugins/car:/usr/local/share/jderobot/gazebo/plugins/f1:/usr/local/share/jderobot/gazebo/plugins/roomba
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/jderobot:/usr/local/share/jderobot/gazebo/plugins/pioneer:/usr/local/share/jderobot/gazebo/plugins/nao:/usr/local/share/jderobot/gazebo/plugins/kinect:/usr/local/share/jderobot/gazebo/plugins/quadrotor:/usr/local/share/jderobot/gazebo/plugins/turtlebot:/usr/local/share/jderobot/gazebo/plugins/flyingkinect:/usr/local/share/jderobot/gazebo/plugins/car:/usr/local/share/jderobot/gazebo/plugins/f1:/usr/local/share/jderobot/gazebo/plugins/roomba
