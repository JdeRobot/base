
if(pcl_openni)

    SET( SOURCE_FILES kinectServer.cpp)

    include_directories(
        ${INTERFACES_CPP_DIR}
        ${LIBS_DIR}/
        ${CMAKE_CURRENT_SOURCE_DIR}
    	${easyiceconfig_INCLUDE_DIRS}
        /usr/include/vtk-5.8
    )

    link_directories(
	${pcl_LIBRARIES}
	${easyiceconfig_LIBRARY_DIRS}
    )
    include_directories(${pcl_INCLUDE_DIR})

    add_executable (pclRGBDServer  ${SOURCE_FILES})

    TARGET_LINK_LIBRARIES(pclRGBDServer 
      ${CMAKE_THREAD_LIBS_INIT}
      ${ZeroCIce_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      JderobotInterfaces
      jderobotutil
      colorspacesmm
      ${openni_LIBRARIES}
      ${Boost_LIBRARIES}
      ${easyiceconfig_LIBRARIES}
    )

endif()
