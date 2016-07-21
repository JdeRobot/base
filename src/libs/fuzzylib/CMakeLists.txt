include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library (fuzzylib STATIC fuzzylib.c)

add_library (fuzzyshare SHARED fuzzylib.c)

set_target_properties(fuzzyshare PROPERTIES OUTPUT_NAME fuzzylib)

### Install
install(TARGETS fuzzylib
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/jderobot
    COMPONENT core
)
