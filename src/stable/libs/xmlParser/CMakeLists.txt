include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++


include_directories( ${LIBS_DIR}/)


SET(xmlParser_CPP xmlReaderParser.cpp xmlWriterParser.cpp)
SET(xmlParser_H xmlReaderParser.h xmlWriterParser.h)

add_library (xmlParser SHARED ${xmlParser_CPP} ${xmlParser_H})
TARGET_LINK_LIBRARIES(xmlParser ${Boost_LIBRARIES} ${libxmlpp_LIBRARIES} )

INSTALL (TARGETS xmlParser DESTINATION lib/jderobot)
INSTALL (FILES ${xmlParser_H} DESTINATION include/jderobot/xmlParser/)
