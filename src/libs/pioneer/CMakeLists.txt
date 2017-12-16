JDEROBOT_ADD_LIBRARY (pioneer STATIC pioneer.c pioneer.h)

JDEROBOT_ADD_LIBRARY (pioneershare SHARED pioneer.c pioneer.h)

JDEROBOT_RENAME_GENERATED_LIB(pioneershare pioneer)

### Install
install(TARGETS pioneer pioneershare
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT pioneer
)

install( FILES pioneer.h
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/pioneer/
    COMPONENT pioneer
)
