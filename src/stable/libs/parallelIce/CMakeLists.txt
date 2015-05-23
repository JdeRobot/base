include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories( ${LIBS_DIR}/)

SET (parallelIce_CPP cameraClient.cpp pointcloudClient.cpp laserClient.cpp)
SET (parallelIce_H cameraClient.h pointcloudClient.h laserClient.h)

add_library (parallelIce SHARED ${parallelIce_CPP} ${parallelIce_H})
TARGET_LINK_LIBRARIES(parallelIce colorspacesmm logger ${Boost_LIBRARIES} ${ZLIB_LIBRARIES})

INSTALL (TARGETS parallelIce DESTINATION lib/jderobot)
INSTALL (FILES ${parallelIce_H} DESTINATION include/jderobot/parallelIce/)
