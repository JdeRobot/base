if (with_pcl)
    include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
    include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
    include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++


    include_directories( ${LIBS_DIR}/)
    set(jderobotViewerSources JderobotViewer.cpp JderobotViewer.h)

    add_library (jderobotViewer STATIC ${jderobotViewerSources})
    TARGET_LINK_LIBRARIES(jderobotViewer ${Boost_LIBRARIES})

    add_library (jderobotViewerShare SHARED ${jderobotViewerSources})
    TARGET_LINK_LIBRARIES(jderobotViewerShare ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})


    set_target_properties(jderobotViewerShare PROPERTIES OUTPUT_NAME jderobotViewer)

    ### Install
    install(TARGETS jderobotViewer jderobotViewerShare
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        COMPONENT viewer
        )

    install( FILES JderobotViewer.h
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/jderobotViewer/
    COMPONENT viewer
)
endif()
