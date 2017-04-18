SET( SOURCE_FILES common.h controller.cpp drawarea.cpp view.h rgbdManualCalibrator.cpp controller.h drawarea.h view.cpp)

include_directories(
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
    ${gl_INCLUDE_DIRS}
    ${glu_INCLUDE_DIRS}
    ${glut_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
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

add_executable (rgbdManualCalibrator ${SOURCE_FILES})

TARGET_LINK_LIBRARIES( rgbdManualCalibrator
 ${GLUT_LIBRARIES}
 ${opencv_LIBRARIES}
 ${OpenCVGUI_LIBRARIES}
 ${gtkmm_LIBRARIES}
 ${libglademm_LIBRARIES}
 ${gthread_LIBRARIES}
 ${libgnomecanvas_LIBRARIES}
 ${libgnomecanvasmm_LIBRARIES}
 ${gtkglextmm_LIBRARIES}
 ${easyiceconfig_LIBRARIES} 
 ${ZeroCIce_LIBRARIES}
 ${OpenCV_LIBRARIES}
 ${gsl_LIBRARIES}
 ${CMAKE_THREAD_LIBS_INIT}
parallelIce
 jderobotutil
 colorspacesmm
 JderobotInterfaces
 progeo
 ${ZLIB_LIBRARIES}    
${resourcelocator_LIBRARIES}
		${GLOG_LIBRARIES}

		)

INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/rgbdManualCalibrator.glade DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/glade)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/rgbdManualCalibrator.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/cam1-mia.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
