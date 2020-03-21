SET(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    "${CMAKE_CURRENT_LIST_DIR}/CMake"
)
SET(CMAKE_PREFIX_PATH  /opt/vc/include /opt/vc/lib)

FIND_PACKAGE(MMAL QUIET)

IF( MMAL_FOUND )
    MESSAGE("-- mmal found")
ELSE()
    MESSAGE("-- ${BoldYellow}mmal not found${ColourReset}")
ENDIF( MMAL_FOUND )
