	SET(SOURCE_FILES
			replayer.cpp
			replayergui.cpp
			utils/SyncController
			interfaces/ReplayerCamera
            interfaces/SyncTask.cpp interfaces/SyncTask.h interfaces/ReplayerPointCloud.cpp interfaces/ReplayerPointCloud.h interfaces/ReplayerLaser.cpp interfaces/ReplayerLaser.h interfaces/ReplayerPose3DEncoders.cpp interfaces/ReplayerPose3DEncoders.h interfaces/ReplayerPose3D.cpp interfaces/ReplayerPose3D.h interfaces/ReplayerEncoders.cpp interfaces/ReplayerEncoders.h utils/ReplayControllerInterface.cpp utils/ReplayControllerInterface.h)

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
    ${GLUT_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OpenCVGUI_LIBRARIES}
    ${gtkmm_LIBRARIES}
    ${libglademm_LIBRARIES}
    ${gtkglextmm_LIBRARIES}
	jderobotutil
    colorspacesmm
    JderobotInterfaces
    ${gsl_LIBRARIES}
    ${easyiceconfig_LIBRARIES} 
    ${ZeroCIce_LIBRARIES}
    ${resourcelocator_LIBRARIES}
	${GLOG_LIBRARIES}
	${ZLIB_LIBRARIES}
	crypto
	${CMAKE_THREAD_LIBS_INIT}

		)

install(TARGETS replayer2
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
    COMPONENT tools
    )
    
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/replayergui.glade DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/glade)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/replayer.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
