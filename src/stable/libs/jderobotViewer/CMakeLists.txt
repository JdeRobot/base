if (with_pcl)
    include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
    include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
    include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++


    include_directories( ${LIBS_DIR}/)
    set(jderobotViewer_Sources_CPP JderobotViewer.cpp)
    set(jderobotViewer_Sources_H JderobotViewer.h)

    add_library (jderobotViewer SHARED ${jderobotViewer_Sources_CPP} ${jderobotViewer_Sources_H})
    TARGET_LINK_LIBRARIES(jderobotViewer ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

    INSTALL (TARGETS jderobotViewer DESTINATION lib/jderobot)
    INSTALL (FILES ${jderobotViewer_H} DESTINATION include/jderobot/jderobotViewer/)
endif()
