
SET( SOURCE_FILES teleoperatorPC.cpp SharedMemory.cpp Gui.cpp controlICE.cpp)

include_directories(
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
    ${gtkmm_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${easyiceconfig_INCLUDE_DIRS}
)

link_directories(
	${easyiceconfig_LIBRARY_DIRS}
)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated" ) # Opciones para el compilador

add_executable (teleoperatorPC  ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(teleoperatorPC 
    JderobotInterfaces
    jderobotutil
    colorspacesmm
    ${OpenCV_LIBRARIES}        
    ${OpenCVGUI_LIBRARIES}
    ${easyiceconfig_LIBRARIES} 
    ${ZeroCIce_LIBRARIES}    
    ${gtkmm_LIBRARIES}  
    ${gthread_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

