
SET(imgAnalyze_CPP geometry.cpp image.cpp linesDetection.cpp)
SET(imgAnalyze_H geometry.h image.h linesDetection.h structs.h)


include_directories( ${LIBS_DIR}/)
add_library (imgAnalyze SHARED ${imgAnalyze_CPP} ${imgAnalyze_H})
TARGET_LINK_LIBRARIES(imgAnalyze progeo cvfast)

INSTALL (TARGETS imgAnalyze DESTINATION lib/jderobot)
INSTALL (FILES ${imgAnalyze_H} DESTINATION include/jderobot/visionlib/imgAnalyze/)
