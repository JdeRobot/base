include_directories(${CMAKE_CURRENT_SOURCE_DIR})

JDEROBOT_ADD_LIBRARY (fuzzylib STATIC fuzzylib.c)

JDEROBOT_ADD_LIBRARY (fuzzyshare SHARED fuzzylib.c)

JDEROBOT_RENAME_GENERATED_LIB(fuzzyshare fuzzylib)

### Install
install(TARGETS fuzzylib fuzzyshare
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT fuzzylib
)

install( FILES fuzzylib.h
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/fuzzylib/
    COMPONENT fuzzylib
)
