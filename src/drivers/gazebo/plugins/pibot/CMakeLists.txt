include_directories(
	${GAZEBO_INCLUDE_DIRS}
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${easyiceconfig_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS}
    ${easyiceconfig_LIBRARY_DIRS}
)

add_library(motorsPIBOT SHARED motors.cc)
target_link_libraries(motorsPIBOT 
	${GAZEBO_libraries} 
	${ZeroCIce_LIBRARIES} 
	colorspacesmm
	JderobotInterfaces
    	${easyiceconfig_LIBRARIES}
)   

add_library(sonarPIBOT SHARED sonar.cc)
target_link_libraries(sonarPIBOT 
	SonarPlugin 
	${GAZEBO_libraries} 
	${ZeroCIce_LIBRARIES} 
	colorspacesmm
   	JderobotInterfaces
	${easyiceconfig_LIBRARIES}
) 

add_library(pose3dPIBOT SHARED pose3d.cc)
target_link_libraries(pose3dPIBOT
	${GAZEBO_libraries} 
	${ZeroCIce_LIBRARIES} 
	colorspacesmm
    	JderobotInterfaces
	${easyiceconfig_LIBRARIES}
) 

add_library(camera_dumpPIBOT SHARED camera_dump.cc)
target_link_libraries(camera_dumpPIBOT ${GAZEBO_libraries} 
	CameraPlugin 	
	${GAZEBO_libraries} 
	${ZeroCIce_LIBRARIES} 
	colorspacesmm
	JderobotInterfaces
	${easyiceconfig_LIBRARIES}
)

add_library(irPIBOT SHARED ir.cc)
target_link_libraries(irPIBOT ${GAZEBO_libraries} 
	CameraPlugin 	
	${GAZEBO_libraries} 
	${ZeroCIce_LIBRARIES} 
	colorspacesmm
	JderobotInterfaces
	${easyiceconfig_LIBRARIES}
)
 
INSTALL (TARGETS motorsPIBOT sonarPIBOT pose3dPIBOT camera_dumpPIBOT irPIBOT
	DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/gazebo/plugins/pibot/ 
	COMPONENT gazebo)

FILE(GLOB_RECURSE CFG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cfg)
INSTALL (FILES ${CFG_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT gazebo)

#MIO endif()
