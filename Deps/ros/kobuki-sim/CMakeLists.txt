find_package(catkin QUIET COMPONENTS
	      kobuki_gazebo
	    )

if(kobuki_gazebo_FOUND)
 message("*** kobuki_gazebo FOUND")
else()
 message("*** kobuki_gazebo NOT FOUND")
endif()

list(APPEND DEPS ros-kinetic-kobuki-gazebo)