if (${QT5_COMPILE})
    
    set(GUI_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/gui)
#    set(CMAKE_AUTOMOC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS} ${Qt5Core_EXECUTABLE_COMPILE_FLAGS} ${Qt5Gui_EXECUTABLE_COMPILE_FLAGS} ${Qt5OpenGL_EXECUTABLE_COMPILE_FLAGS}")


    add_definitions(${Qt5Widgets_DEFINITIONS}
                        ${Qt5Core_DEFINITIONS}
                        ${Qt5Gui_DEFINITIONS}
                        ${Qt5OpenGL_DEFINITIONS}
    )


    SET(HEADERS gui/gui.h gui/threadupdategui.h gui/widget/cameraswidget.h gui/widget/controlvw.h)
    
    SET(SOURCE basic_component.cpp gui/gui.cpp gui/threadupdategui.cpp gui/widget/cameraswidget.cpp gui/widget/controlvw.cpp 
        ${HEADERS}
    )

	qt5_wrap_cpp(qt_SRC ${SOURCE})	

    include_directories(
        ${Qt5Widgets_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Gui_INCLUDE_DIRS}
        ${Qt5OpenGL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
        ${Ice_INCLUDE_DIR}
        ${INTERFACES_CPP_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${LIBS_DIR}
        ${jderobottypes_INCLUDE_DIRS}
        ${comm_INCLUDE_DIRS}
        ${config_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${ZLIB_INCLUDE_DIRS}
    )

    link_directories(
        ${comm_LIBRARY_DIRS}
        ${config_LIBRARY_DIRS}
    )


    add_executable( basic_component ${SOURCE} ${INTERFACES_CPP_DIR} ${qt_SRC})
                   
    target_link_libraries(basic_component
        
        ${Qt5Widgets_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        ${Qt5Gui_LIBRARIES}
        ${Qt5OpenGL_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        ${ZeroCIce_LIBRARIES}
        ${Ice_LIBRARIES}
        ${OPENGL_LIBRARIES} ${GLUT_LIBRARY}
        ${OpenCV_LIBRARIES}
        ${OpenCVGUI_LIBRARIES}
        ${catkin_LIBRARIES}
        ${comm_LIBRARIES}
        ${config_LIBRARIES}
        JderobotInterfaces
        jderobotutil
        colorspacesmm
        ${GLOG_LIBRARIES}
        ${ZLIB_LIBRARIES}
    )

    install(TARGETS basic_component
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
    COMPONENT basic-component
    )
    INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/basic_component.yml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf
    COMPONENT basic-component)
ENDIF()




