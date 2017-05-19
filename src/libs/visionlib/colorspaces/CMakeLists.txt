
add_library (colorspaces STATIC rgb2hsv.c rgb2yuv.c colorspaces.h colorspacesmm.h)

TARGET_LINK_LIBRARIES(colorspaces ${OpenCV_LIBRARIES})

add_library (colorspacesshare SHARED rgb2hsv.c rgb2yuv.c colorspaces.h colorspacesmm.h)

TARGET_LINK_LIBRARIES(colorspacesshare  ${OpenCV_LIBRARIES})

add_library (colorspacesmm SHARED imagecv.cpp imagecv.h uncopyable.h)

TARGET_LINK_LIBRARIES(colorspacesmm ${OpenCV_LIBRARIES} ${OpenCVGUI_LIBRARIES})

set_target_properties(colorspacesshare PROPERTIES OUTPUT_NAME colorspaces)

### Install
install(TARGETS colorspaces colorspacesmm
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/jderobot
    COMPONENT core
)

FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
install( FILES ${HEADERS_FILES}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/visionlib/colorspaces
    COMPONENT core
)
