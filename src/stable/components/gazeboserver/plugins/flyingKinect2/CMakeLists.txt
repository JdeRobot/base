if (PCL_LIBRARIES)

##
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

set( CMAKE_CXX_FLAGS "-Wall" ) # Opciones para el compilador

set(SOURCES
	src/kinect.cc
)

add_library(gazebo_kinect SHARED ${SOURCES})

target_link_libraries(gazebo_kinect
    JderobotInterfaces
    jderobotutil
    ${GAZEBO_libraries} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${ZeroCIce_LIBRARIES}
#    ${easyiceconfig_LIBRARIES}
    easyiceconfig-embedded
)

INSTALL (TARGETS gazebo_kinect DESTINATION share/jderobot/gazebo/plugins/flyingkinect/ COMPONENT core)
FILE(GLOB_RECURSE CFG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cfg)
INSTALL (FILES ${CFG_FILES} DESTINATION share/jderobot/gazebo/plugins/flyingkinect/ COMPONENT core)

else()
   message(WARNING "flyingKinect requirements unmet: missing PCL_LIBRARIES")
endif()
