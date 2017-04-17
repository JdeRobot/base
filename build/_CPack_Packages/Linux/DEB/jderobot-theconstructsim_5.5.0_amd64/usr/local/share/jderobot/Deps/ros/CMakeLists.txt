find_package(roscpp QUIET)

if(roscpp_FOUND)

	MESSAGE("*** ROS LIBRARIES FOUND : ${roscpp_INCLUDE_DIRS}")

    SET (CMAKE_PREFIX_PATH /opt/ros/indigo)

    usePython(2)
    set (CATKIN_BUILD_BINARY_PACKAGE 1) #doesn't add ROS environment files to package

    find_package(catkin REQUIRED COMPONENTS
      rospy
    )

	list(APPEND DEPS ros-indigo-rospy)

#	list(APPEND DEPS_DEV )
else()
	MESSAGE("*** ROS LIBRARIES NOT FOUND")
endif()

