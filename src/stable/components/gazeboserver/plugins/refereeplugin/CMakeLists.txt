include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${INTERFACES_CPP_DIR}
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${INTERFACES_CPP_DIR}
)

add_library(refereeplugin SHARED referee.cc)
target_link_libraries(refereeplugin
	Ice
	IceUtil
    JderobotInterfaces
	${GAZEBO_libraries} 
)

INSTALL (TARGETS refereeplugin DESTINATION share/jderobot/gazebo/plugins/refereeplugin/ )
FILE(GLOB_RECURSE CFG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cfg)
INSTALL (FILES ${CFG_FILES} DESTINATION share/jderobot/gazebo/plugins/refereeplugin/)
