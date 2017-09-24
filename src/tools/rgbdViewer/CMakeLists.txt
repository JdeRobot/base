SET(SOURCE_FILES rgbdViewer.cpp drawarea.cpp rgbdViewergui.cpp myprogeo.cpp)

include_directories(
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/cameras
    ${gtkmm_INCLUDE_DIRS}
    ${libglademm_INCLUDE_DIRS}
    ${gtkglextmm_INCLUDE_DIRS}
    ${resourcelocator_INCLUDE_DIRS}
    ${easyiceconfig_INCLUDE_DIRS}
)

link_directories(
	${resourcelocator_LIBRARY_DIRS}
	${easyiceconfig_LIBRARY_DIRS}
)

add_executable (rgbdViewer ${SOURCE_FILES})


TARGET_LINK_LIBRARIES(rgbdViewer
    ${CMAKE_THREAD_LIBS_INIT} 
    ${GLUT_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${gtkmm_LIBRARIES}
    ${OpenCVGUI_LIBRARIES}
    ${libglademm_LIBRARIES}
    ${gtkglextmm_LIBRARIES}
    colorspacesmm
    JderobotInterfaces
	parallelIce
    jderobotutil
    progeo
	geometry
    pioneer
    ${gsl_LIBRARIES}
    ${easyiceconfig_LIBRARIES} 
    ${ZeroCIce_LIBRARIES}
    ${ZLIB_LIBRARIES}
	${resourcelocator_LIBRARIES}
	${GLOG_LIBRARIES}
	)

INSTALL (TARGETS rgbdViewer DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/ COMPONENT rgbdViewer)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/rgbdViewergui.glade DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/glade COMPONENT rgbdViewer)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/rgbdViewer.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT rgbdViewer)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/camera-0.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT rgbdViewer)
