include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++


include_directories( ${LIBS_DIR}/)

SET(depthLib_CPP DepthFilter.cpp DepthSampler.cpp)
SET(depthLib_H DepthFilter.h DepthSampler.h)

add_library (depthLib SHARED ${depthLib_CPP} ${depthLib_H})
TARGET_LINK_LIBRARIES(depthLib ${Boost_LIBRARIES} ${ZeroCIce_LIBRARIES} ${OpenCV_LIBRARIES})

INSTALL (TARGETS depthLib DESTINATION lib/jderobot)
INSTALL (FILES ${depthLib_H} DESTINATION include/jderobot/depthLib/)
