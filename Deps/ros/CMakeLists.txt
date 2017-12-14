OPTION(ENABLE_ROS "Enable ROS compatibility modules" ON)

if (ENABLE_ROS)
	find_package(roscpp REQUIRED)
	
	if(roscpp_FOUND)
	
	    MESSAGE("*** ROS LIBRARIES FOUND : ${roscpp_INCLUDE_DIRS}")
	
	    SET (CMAKE_PREFIX_PATH /opt/ros/kinetic)
	
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
	
		list(APPEND DEPS ros-kinetic-roscpp ros-kinetic-std-msgs ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-roscpp-core ros-kinetic-rospy ros-kinetic-nav-msgs ros-kinetic-geometry-msgs )

		list_subdirectories( LIST_ROS ${CMAKE_CURRENT_SOURCE_DIR} 1)

		FOREACH (driver ${LIST_ROS})
			if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${driver}/CMakeLists.txt)
				ADD_SUBDIRECTORY (${driver})
			endif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${driver}/CMakeLists.txt)
		ENDFOREACH()

	#	list(APPEND DEPS_DEV )
	else()
		MESSAGE(WARNING "*** ROS LIBRARIES NOT FOUND. ROS SUPPORT WILL BE DISABLED")
	endif()
else()
	SET(roscpp_FOUND FALSE)
endif()

