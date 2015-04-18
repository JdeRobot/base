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
