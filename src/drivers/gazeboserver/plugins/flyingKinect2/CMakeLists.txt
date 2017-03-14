if (PCL_LIBRARIES)

project(gazebo_kinectplugin)


## easyiceconfig customs
set(easyiceconfig_STATIC_LIBRARIES easyiceconfig-embedded)


## Dependencies
include_directories( 
	${GAZEBO_INCLUDE_DIRS}
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
	${GEARBOX_INCLUDE_DIRS}
	/usr/include/vtk-5.8
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${easyiceconfig_INCLUDE_DIRS}
)

link_directories( 
	${GAZEBO_LIBRARY_DIRS}
	${easyiceconfig_LIBRARY_DIRS}
)


## Project
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall" ) # Opciones para el compilador

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

set(SOURCES
	src/kinect.cc

    src/quadrotorsensors.cc
    src/cameraproxy.cc

    src/interfaces/camerai.cpp
    src/interfaces/pushcamerai.cpp
    src/interfaces/cameraibase.cpp

	include/kinect/kinect.hh
	include/kinect/pointcloudi.hpp



)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME}
    JderobotInterfaces
    jderobotutil
    ${GAZEBO_libraries} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${ZeroCIce_LIBRARIES}
    ${easyiceconfig_STATIC_LIBRARIES}
)

INSTALL (TARGETS ${PROJECT_NAME} DESTINATION share/jderobot/gazebo/plugins/flyingkinect/ COMPONENT core)
FILE(GLOB_RECURSE CFG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cfg)
INSTALL (FILES ${CFG_FILES} DESTINATION share/jderobot/gazebo/plugins/flyingkinect/ COMPONENT core)

else()
   message(WARNING "flyingKinect requirements unmet: missing PCL_LIBRARIES")
endif()
