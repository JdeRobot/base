project (dnm4)

cmake_minimum_required(VERSION 2.8)
include(FindPkgConfig)

SET( SOURCE_FILES_AUTOMATA 
	dnm4.cpp
)

pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)

SET( INTERFACES_CPP_DIR /opt/jderobot/include )
SET( LIBS_DIR /usr/local/lib )
SET( JDEROBOT_LIBS_DIR /opt/jderobot/lib )
SET( easyiceconfig_LIBRARIES ${JDEROBOT_LIBS_DIR}/libeasyiceconfig.so)

SET( CMAKE_CXX_FLAGS "-pthread" ) # Opciones para el compilador

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

SET(goocanvasmm_INCLUDE_DIRS
	/usr/include/goocanvasmm-2.0
	/usr/lib/goocanvasmm-2.0/include
	/usr/include/goocanvas-2.0
)

include_directories (
	/opt/jderobot/include/jderobot
	${INTERFACES_CPP_DIR}
	${easyiceconfig_INCLUDE_DIRS}
	${LIBS_DIR}
	${JDEROBOT_LIBS_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
	${GTKMM_INCLUDE_DIRS}
	${goocanvasmm_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

link_directories(${GTKMM_LIBRARY_DIRS} ${easyiceconfig_LIBRARY_DIRS})

add_executable (dnm4 ${SOURCE_FILES_AUTOMATA})

SET(goocanvasmm_LIBRARIES goocanvasmm-2.0 goocanvas-2.0)

TARGET_LINK_LIBRARIES (dnm4
	${GTKMM_LIBRARIES}
	${easyiceconfig_LIBRARIES}
	${goocanvasmm_LIBRARIES}
	${JDEROBOT_LIBS_DIR}/libvisualStateslib.so
	${JDEROBOT_LIBS_DIR}/libJderobotInterfaces.so
	${JDEROBOT_LIBS_DIR}/libjderobotutil.so
	Ice
	IceUtil
)
