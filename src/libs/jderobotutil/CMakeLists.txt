include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++


SET(JDEROBOT_UTILS_SOURCES
        jderobotutil.h
        observer
        paramdict
        time
        uncopyable.h
        interfaceHandlers/CameraTask
        interfaceHandlers/CameraHandler
        interfaceHandlers/RGBDHandler
        utils/CameraUtils
        )

JDEROBOT_ADD_LIBRARY (jderobotutil STATIC ${JDEROBOT_UTILS_SOURCES} )

JDEROBOT_ADD_LIBRARY (jderobotutilshare SHARED ${JDEROBOT_UTILS_SOURCES})

JDEROBOT_RENAME_GENERATED_LIB(jderobotutilshare jderobotutil)

### Install
install(TARGETS jderobotutil jderobotutilshare
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT util
)

FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
install( FILES ${HEADERS_FILES}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/jderobotutil/
    COMPONENT util
)
