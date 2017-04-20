SET( SOURCE_FILES
	navigatorCamera.cpp
	gui.cpp
	sharer.cpp
)

add_definitions(-DGLADE_DIR="${gladedir}")

include_directories(
	${INTERFACES_CPP_DIR}
	${LIBS_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
	${gtkmm_INCLUDE_DIRS}
	${libglademm_INCLUDE_DIRS}
	${gtkglextmm_INCLUDE_DIRS}
	${ZLIB_INCLUDE_DIRS}
	${resourcelocator_INCLUDE_DIRS}
	${easyiceconfig_INCLUDE_DIRS}
)
link_directories(
	${resourcelocator_LIBRARY_DIRS}
	${easyiceconfig_LIBRARY_DIRS}
)


add_executable (navigatorCamera  ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(navigatorCamera
	${CMAKE_THREAD_LIBS_INIT}
	${gtkmm_LIBRARIES}
	${libglademm_LIBRARIES}
	${gtkglextmm_LIBRARIES}
	${OpenCV_LIBRARIES}
	colorspacesmm
	JderobotInterfaces
	parallelIce
	jderobotutil
	${easyiceconfig_LIBRARIES} 
	${ZeroCIce_LIBRARIES}
	${ZLIB_LIBRARIES}
	${resourcelocator_LIBRARIES}
		${GLOG_LIBRARIES}
)

install(TARGETS navigatorCamera
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
    COMPONENT tools
    )

INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/myimage.png DESTINATION /usr/local/share/jderobot/glade/ )
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/navigatorCamera.glade DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/glade )
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/navigatorCamera.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
