
cmake_minimum_required(VERSION 2.8)

SET(SOURCE_CPP_RUNTIME
	gui/codegen/cpp/state.cpp
	gui/codegen/cpp/transition.cpp
	gui/codegen/cpp/temporaltransition.cpp
        gui/codegen/cpp/runtimegui.cpp
)

INCLUDE_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/include)
LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib)

ADD_LIBRARY(visualStatesRunTime SHARED ${SOURCE_CPP_RUNTIME})
TARGET_LINK_LIBRARIES(visualStatesRunTime easyiceconfig Ice JderobotInterfaces IceUtil pthread)

INSTALL(TARGETS visualStatesRunTime DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/visualstates_py/)

#install runtime headers
INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/gui/codegen/cpp/
		DESTINATION ${CMAKE_INSTALL_PREFIX}/include/visualstates_py/
		FILES_MATCHING PATTERN "*.h")

#install python runtime dependencies
INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/gui/codegen/python/
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python3.5/visualStates_py PATTERN __pycache__ EXCLUDE )
