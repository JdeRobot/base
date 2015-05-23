SET(colorspacesmm_CPP imagecv.cpp)
SET(colorspacesmm_H imagecv.h uncopyable.h)

add_library (colorspacesmm SHARED ${colorspacesmm_CPP} ${colorspacesmm_H})
TARGET_LINK_LIBRARIES(colorspacesmm ${OpenCV_LIBRARIES} ${OpenCVGUI_LIBRARIES})

SET(colorspaces_CPP rgb2hsv.c rgb2yuv.c )
SET(colorspaces_H imagecv.h colorspaces.h colorspacesmm.h)

add_library (colorspaces SHARED ${colorspaces_CPP} ${colorspaces_H})
TARGET_LINK_LIBRARIES(colorspaces ${OpenCV_LIBRARIES})

INSTALL (TARGETS colorspaces colorspacesmm DESTINATION lib/jderobot)
INSTALL (FILES ${colorspacesmm_H} DESTINATION include/jderobot/visionlib/colorspacesmm/)
INSTALL (FILES ${colorspaces_H} DESTINATION include/jderobot/visionlib/colorspaces/)
