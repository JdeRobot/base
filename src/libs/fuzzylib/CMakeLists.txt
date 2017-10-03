include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library (fuzzylib STATIC fuzzylib.c)

add_library (fuzzyshare SHARED fuzzylib.c)

set_target_properties(fuzzyshare PROPERTIES OUTPUT_NAME fuzzylib)

### Install
install(TARGETS fuzzylib fuzzyshare
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT fuzzylib
)

install( FILES fuzzylib.h
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/fuzzylib/
    COMPONENT fuzzylib
)
