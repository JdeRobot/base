project (wallFollowercpp)

cmake_minimum_required(VERSION 2.8)
include(FindPkgConfig)

SET( SOURCE_FILES_AUTOMATA 
	wallFollowercpp.cpp
)

pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)

find_package(roscpp QUIET)
if (roscpp_FOUND)
	find_package(catkin REQUIRED COMPONENTS
		roscpp
		std_msgs
		cv_bridge
		image_transport
		nav_msgs
		geometry_msgs
	)
endif()
SET( INTERFACES_CPP_DIR /usr/local/include )
SET( LIBS_DIR /usr/local/lib )
SET( easyiceconfig_LIBRARIES easyiceconfig)
SET(jderobotcomm_LIBRARY_DIRS ${LIBS_DIR}/jderobot)
SET(jderobotcomm_LIBRARIES jderobotcomm)

SET( CMAKE_CXX_FLAGS "-pthread" ) # Opciones para el compilador

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

SET(goocanvasmm_INCLUDE_DIRS
	/usr/include/goocanvasmm-2.0
	/usr/lib/goocanvasmm-2.0/include
	/usr/include/goocanvas-2.0
)

include_directories (
	/usr/local/include/jderobot
	${INTERFACES_CPP_DIR}
	${easyiceconfig_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${GTKMM_INCLUDE_DIRS}
	${goocanvasmm_INCLUDE_DIRS}
	${roscpp_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

link_directories(${GTKMM_LIBRARY_DIRS} ${easyiceconfig_LIBRARY_DIRS} ${jderobotcomm_LIBRARY_DIRS})

add_executable (wallFollowercpp ${SOURCE_FILES_AUTOMATA})

SET(goocanvasmm_LIBRARIES goocanvasmm-2.0 goocanvas-2.0)

TARGET_LINK_LIBRARIES (wallFollowercpp
	${jderobotcomm_LIBRARIES}
	${GTKMM_LIBRARIES}
	${easyiceconfig_LIBRARIES}
	${goocanvasmm_LIBRARIES}
	visualHFSMlib
	JderobotInterfaces
	jderobotutil
	${catkin_LIBRARIES}
	colorspacesmm
	Ice
	IceUtil
	IceStorm
	glog
)
