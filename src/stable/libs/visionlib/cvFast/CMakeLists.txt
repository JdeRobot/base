SET(cvfast_CPP cvfast.cpp)
SET(cvfast_H cvfast.h)

add_library(cvfast SHARED ${cvfast_CPP} ${cvfast_H})
TARGET_LINK_LIBRARIES(cvfast ${OPENGL_LIBRARIES})

INSTALL (TARGETS cvfast DESTINATION lib/jderobot)
INSTALL (FILES ${cvfast_H} DESTINATION include/jderobot/visionlib/cvfast/)
