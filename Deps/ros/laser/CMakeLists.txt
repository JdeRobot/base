
#[[find_package(catkin QUIET COMPONENTS
	      rplidar_ros
	      urg_node
	    )

if(rplidar_ros_FOUND)
 message("*** rplidar FOUND")
else()
 message("*** RPLIDAR NOT FOUND")
endif()

if(urg_node_FOUND)
 message("*** HOKUYO FOUND")
else()
 message("*** HOKUYO NOT FOUND")
endif()
]]
list(APPEND DEPS ros-kinetic-rplidar-ros ros-kinetic-urg-node)