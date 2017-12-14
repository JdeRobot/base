#[[find_package(catkin QUIET COMPONENTS
	      kobuki_gazebo
	    )

if(kobuki_gazebo_FOUND)
 message("*** TURTLEBOT-SIM FOUND")
else()
 message("*** TURTLEBOT-SIM NOT FOUND")
endif()
]]
list(APPEND DEPS ros-kinetic-kobuki-gazebo)