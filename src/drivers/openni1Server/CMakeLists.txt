IF(openni_LIBRARIES AND LIBUSB_INCLUDE_DIRS)

	    SET( SOURCE_FILES openni1Server.cpp myprogeo.cpp )
	    add_definitions(-DGLADE_DIR="${gladedir}")


	    include_directories(
            ${INTERFACES_CPP_DIR}
            ${LIBS_DIR}/
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/cameras
            ${openni_INCLUDE_DIR}
            ${LIBUSB_INCLUDE_DIRS}
       		${easyiceconfig_INCLUDE_DIRS}
		)

		link_directories(
			${easyiceconfig_LIBRARY_DIRS}
		)

	    add_executable (openni1Server ${SOURCE_FILES})

			IF( nite_LIBRARIES)

				set_property(
			  		SOURCE openni1Server.cpp
			   		PROPERTY COMPILE_DEFINITIONS WITH_NITE=1
			   	)
				include_directories(
				    ${nite_INCLUDE_DIR}
		    	)

			TARGET_LINK_LIBRARIES(openni1Server
				  ${OpenCV_LIBRARIES}
				  ${OpenCVGUI_LIBRARIES}
				  ${gsl_LIBRARIES}
				  ${easyiceconfig_LIBRARIES} 
				  ${ZeroCIce_LIBRARIES} 
				  ${openni_LIBRARIES}
				  ${nite_LIBRARIES}
				  ${CMAKE_THREAD_LIBS_INIT}
				  ${LIBUSB_LIBRARIES}
				  ${Boost_LIBRARIES}
				  colorspacesmm
				  JderobotInterfaces
				  jderobotutil
				  progeo
				)
		ELSE()
			TARGET_LINK_LIBRARIES(openni1Server
				  ${OpenCV_LIBRARIES}
				  ${OpenCVGUI_LIBRARIES}
				  ${gsl_LIBRARIES}
				  ${easyiceconfig_LIBRARIES} 
				  ${ZeroCIce_LIBRARIES} 
				  ${openni_LIBRARIES}
				  ${CMAKE_THREAD_LIBS_INIT}
				  ${LIBUSB_LIBRARIES}
				  ${Boost_LIBRARIES}
				  colorspacesmm
				  colorspacesmm
				  JderobotInterfaces
				  jderobotutil
				  progeo
				)
		ENDIF()
ENDIF()
