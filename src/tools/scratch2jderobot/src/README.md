# Execution steps

1. Translate the Scratch program to Python

```
roscd scratch2ros/scripts
python scratch2python example.sb2
```
  
2. Launch the ROBOT or the DRONE (simulation)

**ROS specific**:

* Kobuki robot (empty world):

```
roslaunch kobuki_gazebo kobuki_empty_world.launch --screen
```

* Turtlebot robot (with obstacles):

```
roslaunch turtlebot_gazebo turtlebot_world.launch
```

**ICE specific**:

* Kobuki robot (laberinth):

```
gazebo kobuki-simple.world
```

* Parrot drone (empty world):

```
gazebo ArDrone.world
```

* Parrot drone (cat-mouse world):

```
gazebo ardrone-trees-simple.world
```

3. Execute the translated python program (depending on the type of robot)

```
python main_robot.py
```

or

```
python main_drone.py
```
