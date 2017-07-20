SET(cvFast_CVFAST cvfast.cpp)
add_library(cvfast SHARED ${cvFast_CVFAST})
TARGET_LINK_LIBRARIES(cvfast ${OPENGL_LIBRARIES})


### Install
install(TARGETS cvfast
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT core
)

FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
install( FILES ${HEADERS_FILES}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/visionlib/cvfast
    COMPONENT core
)
