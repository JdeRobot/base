

set(SRC_FILES Logger.h Logger.cpp) 
 
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	include(CheckCXXCompilerFlag)
	CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
	CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  # Si el compilador es superior a 4.6 debemos incluir una opción adicional
	message (STATUS "logger version: ${CMAKE_CXX_COMPILER_VERSION} found")
	if(COMPILER_SUPPORTS_CXX11)
    	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	elseif(COMPILER_SUPPORTS_CXX0X)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	else()
		    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
	endif()

endif()

JDEROBOT_ADD_LIBRARY(logger SHARED ${SRC_FILES})

target_link_libraries(logger ${Boost_LIBRARIES})

#SET_PROPERTY(TARGET logger PROPERTY SOVERSION 0.1.0)

### Install
install(TARGETS logger
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT logger
)
install( FILES Logger.h
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/logger/
    COMPONENT logger
)
 
