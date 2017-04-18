	SET(SOURCE_FILES replayer.cpp replayergui.cpp control.cpp)

add_definitions(-DGLADE_DIR="${gladedir}")

include_directories(
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/cameras
    ${gtkmm_INCLUDE_DIRS}
    ${libglademm_INCLUDE_DIRS}
    ${gtkglextmm_INCLUDE_DIRS}
    ${easyiceconfig_INCLUDE_DIRS}
    ${resourcelocator_INCLUDE_DIRS}
)

link_directories(
	${easyiceconfig_LIBRARY_DIRS}
	${resourcelocator_LIBRARY_DIRS}
)

add_executable (replayer2 ${SOURCE_FILES})


TARGET_LINK_LIBRARIES(replayer2
    ns
    logger
    ${CMAKE_THREAD_LIBS_INIT} 
    ${GLUT_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OpenCVGUI_LIBRARIES}
    ${gtkmm_LIBRARIES}
    ${libglademm_LIBRARIES}
    ${gtkglextmm_LIBRARIES}
    colorspacesmm
    JderobotInterfaces
    jderobotutil
    ${gsl_LIBRARIES}
    ${easyiceconfig_LIBRARIES} 
    ${ZeroCIce_LIBRARIES}
    ${resourcelocator_LIBRARIES}
		${GLOG_LIBRARIES}

		)


INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/replayergui.glade DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/glade)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/replayer.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
