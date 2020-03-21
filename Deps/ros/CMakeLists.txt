OPTION(ENABLE_ROS "Enable ROS compatibility modules" ON)

if (ENABLE_ROS)
	find_package(roscpp REQUIRED)
	
	if(roscpp_FOUND)
	
	    MESSAGE(STATUS "ROS libraries found")
	
	    SET (CMAKE_PREFIX_PATH /opt/ros/$ENV{ROS_DISTRO})
	
	    usePython(2)
	    set (CATKIN_BUILD_BINARY_PACKAGE 1) #doesn't add ROS environment files to package

	    find_package(catkin REQUIRED COMPONENTS
	      roscpp
	      std_msgs
	      cv_bridge
	      image_transport
	      nav_msgs
	      geometry_msgs
	    )
	
		list(APPEND DEPS ros-melodic-roscpp ros-melodic-std-msgs ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-roscpp-core ros-melodic-rospy ros-melodic-nav-msgs ros-melodic-geometry-msgs ros-melodic-mavros ros-melodic-gazebo-plugins)

		ADD_SUBDIRECTORY (${CMAKE_CURRENT_SOURCE_DIR}/Deps/ros/laser)
		ADD_SUBDIRECTORY (${CMAKE_CURRENT_SOURCE_DIR}/Deps/ros/kobuki-real)
		ADD_SUBDIRECTORY (${CMAKE_CURRENT_SOURCE_DIR}/Deps/ros/kobuki-sim)


	#	list(APPEND DEPS_DEV )
	else()
		MESSAGE(STATUS "${BoldYellow}ROS libraries not found. ${BoldRed}ROS SUPPORT WILL BE DISABLED${ColourReset}")
	endif()
else()
	SET(roscpp_FOUND FALSE)
endif()

