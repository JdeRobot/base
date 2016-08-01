SET( SOURCE_FILES geometry.cpp image.cpp linesDetection.cpp)
include_directories( ${LIBS_DIR}/)
add_library (imgAnalyze SHARED ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(imgAnalyze progeo cvfast)

### Install
install(TARGETS imgAnalyze
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/jderobot
    COMPONENT core
)
