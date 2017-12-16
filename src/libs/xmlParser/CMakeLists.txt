include_directories(${SLICE_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${INTERFACES_CPP_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++
include_directories(${LIBS_DIR}) # Aquí se alojan las cabeceras de las interfaces ICE en C++


include_directories( ${LIBS_DIR}/)

JDEROBOT_ADD_LIBRARY (xmlParser STATIC xmlReaderParser.cpp  xmlReaderParser.h  xmlWriterParser.cpp  xmlWriterParser.h)
TARGET_LINK_LIBRARIES(xmlParser ${Boost_LIBRARIES})

JDEROBOT_ADD_LIBRARY (xmlParsershare SHARED xmlReaderParser.cpp  xmlReaderParser.h  xmlWriterParser.cpp  xmlWriterParser.h)
TARGET_LINK_LIBRARIES(xmlParsershare ${Boost_LIBRARIES} ${libxmlpp_LIBRARIES} )

JDEROBOT_RENAME_GENERATED_LIB(xmlParsershare xmlParser)

### Install
install(TARGETS xmlParser xmlParsershare
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    COMPONENT xmlparser
)

FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
install( FILES ${HEADERS_FILES}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/xmlParser
    COMPONENT xmlparser
)
