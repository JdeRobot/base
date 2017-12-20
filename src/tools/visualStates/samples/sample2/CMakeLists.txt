project(sample2)

cmake_minimum_required(VERSION 2.8)

SET(SOURCE_FILES
	sample2.cpp)

SET(JDEROBOT_INSTALL_PATH /opt/jderobot)

        
SET(JDEROBOT_INCLUDE_DIR ${JDEROBOT_INSTALL_PATH}/include)
SET(VISUALSTATE_RUNTIME_INCLUDE_DIR ${JDEROBOT_INSTALL_PATH}/include/visualstates_py)

SET(JDEROBOT_LIBS_DIR ${JDEROBOT_INSTALL_PATH}/lib)
SET(VISUALSTATE_RUNTIME_LIBS_DIR ${JDEROBOT_INSTALL_PATH}/lib/visualstates_py)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
        
include_directories(
    ${JDEROBOT_INCLUDE_DIR}
    ${VISUALSTATE_RUNTIME_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

link_directories(
    ${JDEROBOT_LIBS_DIR}
    ${VISUALSTATE_RUNTIME_LIBS_DIR}
)

add_executable(sample2 ${SOURCE_FILES})

target_link_libraries(	sample2
visualStatesRunTime
    config
    comm
    JderobotInterfaces
    jderobotutil
    colorspacesmm
    pthread
    Ice
    IceUtil
    IceStorm
    glog
)
