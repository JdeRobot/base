include_directories(
	${GAZEBO_INCLUDE_DIRS}
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)
#boost_system 
add_library(ball SHARED ball.cc)
target_link_libraries(ball 
	${GAZEBO_libraries} 
)


INSTALL (TARGETS ball DESTINATION share/jderobot/gazebo/plugins/ball/ COMPONENT gazebo)
FILE(GLOB_RECURSE CFG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cfg)
INSTALL (FILES ${CFG_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf/ COMPONENT gazebo)





