SET(cvFast_CVFAST cvfast.cpp)
add_library(cvfast SHARED ${cvFast_CVFAST})
TARGET_LINK_LIBRARIES(cvfast ${OPENGL_LIBRARIES})


### Install
install(TARGETS cvfast
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/jderobot
    COMPONENT core
)
