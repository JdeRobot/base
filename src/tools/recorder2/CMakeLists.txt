SET(SOURCE_FILES
		recorder2.cpp
		recordergui
		pools/PoolWriteImages
		pools/poolWritePose3dEncoders
		pools/poolWriteLasers
		pools/poolWritePointCloud
		pools/poolWriteEncoders
		pools/poolWritePose3d
		buffer/RingBuffer
		pools/RecorderPool
		pools/PoolsManager
		buffer/RecorderInterface
        pools/PoolPaths
		)

add_definitions(-DGLADE_DIR="${gladedir}")

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

add_executable (recorder2
${SOURCE_FILES})


TARGET_LINK_LIBRARIES(recorder2
    ${CMAKE_THREAD_LIBS_INIT} 
    ${GLUT_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OpenCVGUI_LIBRARIES}
    ${gtkmm_LIBRARIES}
    ${libglademm_LIBRARIES}
    ${gtkglextmm_LIBRARIES}
    ${gsl_LIBRARIES}
    ${ZeroCIce_LIBRARIES}
    ${Boost_LIBRARIES}
    ${easyiceconfig_LIBRARIES} 
    colorspacesmm
    JderobotInterfaces
    jderobotutil
	ns
    ${resourcelocator_LIBRARIES}
	${GLOG_LIBRARIES}
)



INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/recordergui.glade DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/glade)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/recorder.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
