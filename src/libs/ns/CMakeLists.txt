include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories( ${LIBS_DIR}/)


set(SRC_FILES ns.h ns.cpp) 
 
JDEROBOT_ADD_LIBRARY(ns SHARED ${SRC_FILES})

target_link_libraries(ns 
    JderobotInterfaces
	logger
	${Boost_LIBRARIES}
	${ZeroCIce_LIBRARIES}
)

#SET_PROPERTY(TARGET ns PROPERTY SOVERSION 0.1.0)
 
### Install
install(TARGETS ns
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT ns
)

install( FILES ns.h
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/ns/
    COMPONENT ns
)
