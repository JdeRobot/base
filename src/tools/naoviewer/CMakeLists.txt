IF (goocanvasmm_INCLUDE_DIRS)

SET( SOURCE_FILES_NAOOPERATOR sensors.cpp control.cpp naooperator.cpp displayer.cpp main.cpp )

include_directories (
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${gtkmm3_INCLUDE_DIRS}
    ${gtk3_INCLUDE_DIRS}
    ${goocanvasmm_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${resourcelocator_INCLUDE_DIRS}
)

add_executable (naooperator ${SOURCE_FILES_NAOOPERATOR})

TARGET_LINK_LIBRARIES ( naooperator 
    JderobotInterfaces
    jderobotutil
    ${gtkmm3_LIBRARIES}
    ${gtk3_LIBRARIES}
    ${goocanvasmm_LIBRARIES}
    ${ZeroCIce_LIBRARIES}
    ${OpenCV_LIBRARIES}
    "dl"
    "rt"
    colorspacesmm
    ${CMAKE_THREAD_LIBS_INIT}
    ${resourcelocator_LIBRARIES}
        ${GLOG_LIBRARIES}
        )

    install(TARGETS naooperator
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
    COMPONENT tools
    )

INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/main_gui.glade DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/glade)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/naooperator.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)

ENDIF()


