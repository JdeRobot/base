SET( SOURCE_FILES geometry.cpp image.cpp linesDetection.cpp)
include_directories( ${LIBS_DIR}/)
JDEROBOT_ADD_LIBRARY (imgAnalyze SHARED ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(imgAnalyze progeo cvfast)

### Install
install(TARGETS imgAnalyze
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT imganalyze
)

FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
install( FILES ${HEADERS_FILES}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/visionlib/imgAnalyze
    COMPONENT imganalyze
)
