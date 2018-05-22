
JDEROBOT_ADD_LIBRARY (colorspaces STATIC rgb2hsv.c rgb2yuv.c colorspaces.h colorspacesmm.h)

TARGET_LINK_LIBRARIES(colorspaces ${OpenCV_LIBRARIES})

JDEROBOT_ADD_LIBRARY (colorspacesshare SHARED rgb2hsv.c rgb2yuv.c colorspaces.h colorspacesmm.h)

TARGET_LINK_LIBRARIES(colorspacesshare  ${OpenCV_LIBRARIES})

JDEROBOT_ADD_LIBRARY (colorspacesmm SHARED imagecv.cpp imagecv.h uncopyable.h)

TARGET_LINK_LIBRARIES(colorspacesmm ${OpenCV_LIBRARIES} ${OpenCVGUI_LIBRARIES})

JDEROBOT_RENAME_GENERATED_LIB(colorspacesshare colorspaces)

### Install
install(TARGETS colorspaces colorspacesmm colorspacesshare
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT colorspaces
)

FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
install( FILES ${HEADERS_FILES}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/visionlib/colorspaces
    COMPONENT colorspaces
)
