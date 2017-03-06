find_package(roscpp QUIET)

if(roscpp_FOUND)

	MESSAGE("*** ROS LIBRARIES FOUND : ${roscpp_INCLUDE_DIRS}")

    SET (CMAKE_PREFIX_PATH /opt/ros/kinetic)

	list(APPEND DEPS ros-kinetic-roscpp ros-kinetic-std-msgs ros-kinetic-message-generation ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-roscpp-core ros-kinetic-rospy)

#	list(APPEND DEPS_DEV )
else()
	MESSAGE("*** ROS LIBRARIES NOT FOUND")
endif()

