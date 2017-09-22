cmake_minimum_required(VERSION 2.8)
project(jderobotconfig)

### Project config
include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}/include
)

configure_file(
    include/jderobot/config/hardcodedlocations.h.in
    include/jderobot/config/hardcodedlocations.h
    @ONLY
)

set(HEADERS
    include/jderobot/config/config.h
    ${CMAKE_CURRENT_BINARY_DIR}/include/jderobot/config/hardcodedlocations.h
    include/jderobot/config/loader.hpp
    include/jderobot/config/debug.hpp

    include/jderobot/config/stdutils.hpp
)

set(SOURCES
    src/loader.cpp
)
# set_property(SOURCE src/loader.cpp PROPERTY COMPILE_FLAGS "-std=c++0x")

## Adding shared library for common usage
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

## Adding static library for single .so configurations
# since target is a shared library, -fPIC must be provided
add_library(${PROJECT_NAME}-embedded STATIC ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME}-embedded  ${YAML_CPP_LIBRARIES})
set_property(TARGET ${PROJECT_NAME}-embedded PROPERTY POSITION_INDEPENDENT_CODE 1)

## Export library variables (like find_package)
set(${PROJECT_NAME}_FOUND 1 CACHE BOOL "Find(${PROJECT_NAME})")
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE PATH "Find(${PROJECT_NAME})")
set(${PROJECT_NAME}_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Find(${PROJECT_NAME})")
set(${PROJECT_NAME}_LIBRARIES "${PROJECT_NAME}" CACHE STRINGS "Find(${PROJECT_NAME})")


## demo
add_executable(config_demo src/demo/demo.cpp ${HEADERS})
target_link_libraries(config_demo  ${YAML_CPP_LIBRARIES} ${PROJECT_NAME})
configure_file(src/demo/ice.cfg ice.cfg)
# set_target_properties(config_demo PROPERTIES COMPILE_FLAGS "-std=c++0x")

## tests
#add_executable(config_test_stdutils src/tests/test_stdutils.cpp)
# set_target_properties(config_test_stdutils PROPERTIES COMPILE_FLAGS "-std=c++0x")

#add_executable(config_test_loader src/tests/test_loader.cpp)
#target_link_libraries(config_test_loader ${PROJECT_NAME}  ${YAML_CPP_LIBRARIES})
# set_target_properties(config_test_loader PROPERTIES COMPILE_FLAGS "-std=c++0x")


### Install
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}-embedded
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT core
)
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    COMPONENT core
)
