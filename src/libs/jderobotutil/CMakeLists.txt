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

add_library (jderobotutil STATIC ${JDEROBOT_UTILS_SOURCES} )

add_library (jderobotutilshare SHARED ${JDEROBOT_UTILS_SOURCES})

set_target_properties(jderobotutilshare PROPERTIES OUTPUT_NAME jderobotutil)

### Install
install(TARGETS jderobotutil jderobotutilshare
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/jderobot
    COMPONENT core
)

FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
install( FILES ${HEADERS_FILES}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/jderobotutil/
    COMPONENT core
)
