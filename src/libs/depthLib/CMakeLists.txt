include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++


include_directories( ${LIBS_DIR}/)

add_library (depthLib STATIC DepthFilter.cpp  DepthFilter.h  DepthSampler.cpp  DepthSampler.h)
TARGET_LINK_LIBRARIES(depthLib ${Boost_LIBRARIES})

add_library (depthLibshare SHARED DepthFilter.cpp  DepthFilter.h  DepthSampler.cpp  DepthSampler.h)
TARGET_LINK_LIBRARIES(depthLibshare ${Boost_LIBRARIES} ${ZeroCIce_LIBRARIES} ${OpenCV_LIBRARIES})


set_target_properties(depthLibshare PROPERTIES OUTPUT_NAME depthLib)

### Install
install(TARGETS depthLib depthLibshare
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/jderobot
    COMPONENT core
)

FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
install( FILES ${HEADERS_FILES}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/depthLib/
    COMPONENT core
)
