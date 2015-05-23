include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories( ${LIBS_DIR}/)


SET(ns_CPP ns.cpp)
SET(ns_H ns.h)

 
ADD_LIBRARY(ns SHARED ${ns_CPP} ${ns_H})

target_link_libraries(ns 
    JderobotInterfaces
	${Boost_LIBRARIES}
	${ZeroCIce_LIBRARIES}
)

SET_PROPERTY(TARGET ns PROPERTY SOVERSION 0.1.0)
 
INSTALL (TARGETS ns DESTINATION lib/jderobot)
INSTALL (FILES ${ns_H} DESTINATION include/jderobot/ns/)
