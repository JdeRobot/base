cmake_minimum_required(VERSION 2.8)
project(jderobottypes)


### Project config
include_directories(
    include
)

set(HEADERS
    include/jderobot/types/laserData.h
    include/jderobot/types/image.h
    include/jderobot/types/pose3d.h
    include/jderobot/types/rgbd.h
    include/jderobot/types/navdataData.h
    include/jderobot/types/cmdvel.h
)


## Export library variables (like find_package)
set(${PROJECT_NAME}_FOUND 1 CACHE BOOL "Find(${PROJECT_NAME})")
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE PATH "Find(${PROJECT_NAME})")
set(${PROJECT_NAME}_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Find(${PROJECT_NAME})")
set(${PROJECT_NAME}_LIBRARIES "${PROJECT_NAME}" CACHE STRINGS "Find(${PROJECT_NAME})")



### Install
#install(TARGETS ${PROJECT_NAME}
#    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/jderobot
#    COMPONENT core
#)
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    COMPONENT types
)
