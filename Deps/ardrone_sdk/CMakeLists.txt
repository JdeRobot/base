cmake_minimum_required(VERSION 2.8)

# https://cmake.org/cmake/help/v3.0/module/FindPkgConfig.html
include(FindPkgConfig)
PKG_SEARCH_MODULE(ardronelib OPTIONAL ardronelib)

if (ardronelib_FOUND)
	message(STATUS "ardronelib v.${ardronelib_VERSION} found")
	# message(STATUS "ardronelib libraries: ${ardronelib_LIBRARIES}")
	SET(ARDRONE_ALL_DEPS true)
	list(APPEND DEPS ardronelib)
else()
	message(STATUS "${BoldYellow}ardronelib NOT FOUND${ColourReset}")
endif()
