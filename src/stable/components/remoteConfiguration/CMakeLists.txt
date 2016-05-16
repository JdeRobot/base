

IF (libxmlpp_INCLUDE_DIRS) 

	SET(SOURCE_FILES remoteConfiguration.cpp myparser.cpp)

	add_definitions(-DGLADE_DIR="${gladedir}")

	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated" ) # Opciones para el compilador

	include_directories(
		${INTERFACES_CPP_DIR}
		${LIBS_DIR}/
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/cameras
		${gtkmm_INCLUDE_DIRS}
        ${libglademm_INCLUDE_DIRS}
        ${gtkglextmm_INCLUDE_DIRS}
    	${easyiceconfig_INCLUDE_DIRS}
	)

	link_directories(
		${easyiceconfig_LIBRARY_DIRS}
	)

	add_executable (remoteConfiguration ${SOURCE_FILES})


	TARGET_LINK_LIBRARIES(remoteConfiguration

		 ${gsl_LIBRARIES}
		  ${libxmlpp_LIBRARIES}
		  ${gtkglextmm_LIBRARIES}
		  ${libglademm_LIBRARIES}
		  ${gtkmm_LIBRARIES}
		  JderobotInterfaces
		  jderobotutil
		${CMAKE_THREAD_LIBS_INIT} 
		${easyiceconfig_LIBRARIES} 
		${ZeroCIce_LIBRARIES}
	)
endif()
