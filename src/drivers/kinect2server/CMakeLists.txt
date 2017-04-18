IF(LIBFREENECT_PATH)
	SET( kinect2Server_SOURCE_FILES kinect2Server.cpp )
    include(FindPkgConfig)
    find_package(freenect2 REQUIRED)
    add_executable (kinect2Server  ${kinect2Server_SOURCE_FILES})



    include_directories(
        ${INTERFACES_CPP_DIR}
    	${easyiceconfig_INCLUDE_DIRS}
        ${LIBS_DIR}/ 
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/cameras
		${NITE2_INC}
		${ZLIB_INCLUDE_DIRS}
        ${freenect2_INCLUDE_DIRS}
/usr/local/include/libfreenect2/tinythread/
    )

link_directories(
	${easyiceconfig_LIBRARY_DIRS}
)

    TARGET_LINK_LIBRARIES(kinect2Server
        ${CMAKE_THREAD_LIBS_INIT}
        ${ZeroCIce_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OpenCVGUI_LIBRARIES} 
        ${gsl_LIBRARIES}
        logger
        ns
        jderobotutil
        geometry
        colorspacesmm
        JderobotInterfaces
        ${Boost_LIBRARIES}
    	${easyiceconfig_LIBRARIES}
        ${LIBXML2_LIBRARIES}
        ${ZLIB_LIBRARIES}
        ${freenect2_LIBRARIES}
    )
    INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/kinect2server.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
ELSE()
    message("Disabled")
ENDIF()

