IF(QT5_COMPILE)

    set(GUI_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/gui)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS} ${Qt5Core_EXECUTABLE_COMPILE_FLAGS} ${Qt5Gui_EXECUTABLE_COMPILE_FLAGS} ${Qt5Svg_EXECUTABLE_COMPILE_FLAGS}")

    set(SOURCES sensors/threadsensors.cpp
                gui/threadgui.cpp
                gui/mainwindow.cpp
                gui/velwidget.cpp
                gui/datawidget.cpp
                src/main.cpp
                sensors/sensors.cpp
                resources/images.cpp
    )
    set(HEADERS sensors/threadsensors.h
                gui/threadgui.h
                gui/mainwindow.h
                gui/velwidget.h
                gui/datawidget.h
    )

    include_directories(${Qt5Widgets_INCLUDE_DIRS}
                        ${Qt5Core_INCLUDE_DIRS}
                        ${Qt5Gui_INCLUDE_DIRS}
                        ${Qt5Svg_INCLUDE_DIRS}
                        ${OpenCV_INCLUDE_DIR}
                        ${Ice_INCLUDE_DIR}
                        ${INTERFACES_CPP_DIR}
                        ${easyiceconfig_INCLUDE_DIRS}
    )
    link_directories(${JDE_LIBS}
                        ${easyiceconfig_LIBRARY_DIRS}
    )
    add_definitions(${Qt5Widgets_DEFINITIONS}
                        ${Qt5Core_DEFINITIONS}
                        ${Qt5Gui_DEFINITIONS}
                        ${Qt5Svg_DEFINITIONS}
    )


    add_executable(uav_viewer
                    ${SOURCES}
                    ${HEADERS}
                    ${INTERFACES_CPP_DIR}
    )
    target_link_libraries(uav_viewer
                            ${Qt5Widgets_LIBRARIES}
                            ${Qt5Core_LIBRARIES}
                            ${Qt5Gui_LIBRARIES}
                            ${Qt5Svg_LIBRARIES}
                            ${CMAKE_THREAD_LIBS_INIT}
                            ${OpenCV_LIBRARIES}
                            ${OpenCVGUI_LIBRARIES}
                            ${Ice_LIBRARIES}
                            ${easyiceconfig_LIBRARIES}
                            ${ZeroCIce_LIBRARIES}
                            JderobotInterfaces
                            jderobotutil
                            qfi
            ${GLOG_LIBRARIES}

            )

    INSTALL (TARGETS uav_viewer DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/ COMPONENT uav-viewer)
    INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/uav_viewer.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT uav-viewer)
    INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/uav_viewer_simulated.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT uav-viewer)

ENDIF()

