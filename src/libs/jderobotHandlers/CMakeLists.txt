include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++


include_directories( ${LIBS_DIR}/)

SET (jderobotHandlers_SOURCES common/CommonHandler ReplayControlerClientHDL)

add_library (jderobotHandlers STATIC ${jderobotHandlers_SOURCES} )
#TARGET_LINK_LIBRARIES(jderobotHandlers colorspacesmm logger ${Boost_LIBRARIES})

add_library (jderobotHandlersshare SHARED ${jderobotHandlers_SOURCES})
TARGET_LINK_LIBRARIES(jderobotHandlersshare ${Boost_LIBRARIES} ${Ice_LIBRARIES})


set_target_properties(jderobotHandlersshare PROPERTIES OUTPUT_NAME jderobotHandlers)

### Install
install(TARGETS jderobotHandlers jderobotHandlersshare
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/jderobot
    COMPONENT core
)
