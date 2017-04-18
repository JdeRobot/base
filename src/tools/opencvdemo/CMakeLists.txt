SET( SOURCE_FILES  opencvdemo.cpp viewer.cpp)


include_directories(
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${gtkmm_INCLUDE_DIRS}
    ${libglademm_INCLUDE_DIRS}
    ${resourcelocator_INCLUDE_DIRS}
    ${easyiceconfig_INCLUDE_DIRS}
)

link_directories(
	${resourcelocator_LIBRARY_DIRS}
	${easyiceconfig_LIBRARY_DIRS}
)

add_executable (opencvdemo ${SOURCE_FILES})


TARGET_LINK_LIBRARIES(opencvdemo
    ${CMAKE_THREAD_LIBS_INIT} 
    ${OpenCV_LIBRARIES}
    ${OpenCVGUI_LIBRARIES}
    ${gtkmm_LIBRARIES}
    ${libglademm_LIBRARIES}
    colorspacesmm
    JderobotInterfaces
    jderobotutil
#    ${Gearbox_LIBRARIES}
    ${easyiceconfig_LIBRARIES} 
    ${ZeroCIce_LIBRARIES}
    ${resourcelocator_LIBRARIES}
		${GLOG_LIBRARIES}

		)



INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/opencvdemo.glade DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/glade)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/opencvdemo.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
