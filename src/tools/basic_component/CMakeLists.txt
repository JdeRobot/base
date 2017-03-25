if (${QT5_COMPILE})
    
    set(GUI_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/gui)
    set(CMAKE_AUTOMOC ON)
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

    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated" )

    include_directories(
        ${Qt5Widgets_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Gui_INCLUDE_DIRS}
        ${Qt5OpenGL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
        ${Ice_INCLUDE_DIR}
        ${INTERFACES_CPP_DIR}
        ${LIBS_DIR}/
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${easyiceconfig_INCLUDE_DIRS}
        ${ZLIB_INCLUDE_DIRS}


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

    link_directories(${JDE_LIBS}
        ${easyiceconfig_LIBRARY_DIRS}
    )

    add_executable( basic_component ${SOURCE} ${INTERFACES_CPP_DIR})
                   
    target_link_libraries(basic_component
        ${Qt5Widgets_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        ${Qt5Gui_LIBRARIES}
        ${Qt5OpenGL_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        ${OpenCV_LIBRARIES}
        ${easyiceconfig_LIBRARIES}
        ${ZeroCIce_LIBRARIES}
        ${Ice_LIBRARIES}
        ${OPENGL_LIBRARIES} ${GLUT_LIBRARY}
        ${ZLIB_LIBRARIES}
        colorspacesmm
        JderobotInterfaces
        jderobotutil
        parallelIce


        ${CMAKE_THREAD_LIBS_INIT} 
    ${OpenCV_LIBRARIES}
    ${OpenCVGUI_LIBRARIES}
    ${gtkmm_LIBRARIES}
    ${libglademm_LIBRARIES}
    colorspacesmm
    JderobotInterfaces
    parallelIce
    ${easyiceconfig_LIBRARIES} 
    ${ZeroCIce_LIBRARIES}
    ${ZLIB_LIBRARIES}
${CMAKE_THREAD_LIBS_INIT}
    ${resourcelocator_LIBRARIES}
            ${GLOG_LIBRARIES}
    )
ENDIF()

