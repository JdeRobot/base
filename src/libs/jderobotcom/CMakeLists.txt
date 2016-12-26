cmake_minimum_required(VERSION 2.8)
project(jderobotcom)

list(APPEND Ice_LIBRARIES Ice IceUtil)

### Project config
include_directories(
    include
    ${jderobottypes_INCLUDE_DIRS}
    ${Ice_INCLUDE_DIR}
    ${LIBS_DIR}
)

message(******************************)
message(jderobot_types = ${jderobottypes_INCLUDE_DIRS})
message(******************************)



set(HEADERS
    include/jderobot/com/laserClient.hpp
    include/jderobot/com/interfaces/laserClient.hpp
    include/jderobot/com/ice/laserIceClient.hpp
)

set(SOURCES
    src/laserClient.cpp
    src/ice/laserIceClient.cpp
)

## Adding shared library for common usage
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${Ice_LIBRARIES} logger ${Boost_LIBRARIES} JderobotInterfaces)

## Adding static library for single .so configurations
# since target is a shared library, -fPIC must be provided
add_library(${PROJECT_NAME}-embedded STATIC ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME}-embedded ${Ice_LIBRARIES} logger ${Boost_LIBRARIES} JderobotInterfaces)
set_property(TARGET ${PROJECT_NAME}-embedded PROPERTY POSITION_INDEPENDENT_CODE 1)



## Export library variables (like find_package)
set(${PROJECT_NAME}_FOUND 1 CACHE BOOL "Find(${PROJECT_NAME})")
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE PATH "Find(${PROJECT_NAME})")
set(${PROJECT_NAME}_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Find(${PROJECT_NAME})")
set(${PROJECT_NAME}_LIBRARIES "${PROJECT_NAME}" CACHE STRINGS "Find(${PROJECT_NAME})")



### Install
install(TARGETS ${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/jderobot
    COMPONENT core
)
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    COMPONENT core
)
