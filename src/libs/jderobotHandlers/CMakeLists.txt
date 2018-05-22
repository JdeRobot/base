include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++


include_directories( ${LIBS_DIR}/)

SET (jderobotHandlers_SOURCES common/CommonHandler ReplayControlerClientHDL)

JDEROBOT_ADD_LIBRARY (jderobotHandlers STATIC ${jderobotHandlers_SOURCES} )
#TARGET_LINK_LIBRARIES(jderobotHandlers colorspacesmm logger ${Boost_LIBRARIES})

JDEROBOT_ADD_LIBRARY (jderobotHandlersshare SHARED ${jderobotHandlers_SOURCES})
TARGET_LINK_LIBRARIES(jderobotHandlersshare ${Boost_LIBRARIES} ${Ice_LIBRARIES})


JDEROBOT_RENAME_GENERATED_LIB(jderobotHandlersshare jderobotHandlers)

### Install
install(TARGETS jderobotHandlers jderobotHandlersshare
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT handlers
)

FILE(GLOB_RECURSE HANDLERS_HEADERS_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h)

install( FILES common/CommonHandler.h
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/jderobotHandlers/common/
    COMPONENT handlers
)

install( FILES ReplayControlerClientHDL.h
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/jderobotHandlers/
    COMPONENT handlers
)

