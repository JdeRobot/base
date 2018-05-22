include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++


include_directories( ${LIBS_DIR}/)

JDEROBOT_ADD_LIBRARY (parallelIce STATIC cameraClient.cpp  cameraClient.h  pointcloudClient.cpp  pointcloudClient.h laserClient.cpp laserClient.h motorsClient.cpp motorsClient.h)
TARGET_LINK_LIBRARIES(parallelIce colorspacesmm logger ${Boost_LIBRARIES})

JDEROBOT_ADD_LIBRARY (parallelIceshare SHARED cameraClient.cpp  cameraClient.h  pointcloudClient.cpp  pointcloudClient.h laserClient.cpp laserClient.h motorsClient.cpp motorsClient.h)
TARGET_LINK_LIBRARIES(parallelIceshare colorspacesmm logger ${Boost_LIBRARIES} ${ZLIB_LIBRARIES})


JDEROBOT_RENAME_GENERATED_LIB(parallelIceshare parallelIce)

### Install
install(TARGETS parallelIce parallelIceshare
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT parallelice
)


FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
install( FILES ${HEADERS_FILES}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/parallelIce/
    COMPONENT parallelice
)
