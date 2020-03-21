FIND_PATH( xerces_INCLUDE_DIR NAMES libxml/parser.h  PATHS ENV C++LIB ENV PATH PATH_SUFFIXES libxml2)

IF( xerces_INCLUDE_DIR )
    FIND_LIBRARY( xerces_LIBRARIES NAMES xml2  PATHS ENV C++LIB ENV PATH PATH_SUFFIXES /usr/lib)
    IF( xerces_LIBRARIES )
		MESSAGE (STATUS "Xerces found")
		include_directories(${xerces_INCLUDE_DIR})
		link_directories(${xerces_LIBRARIES})
		list(APPEND DEPS libxerces-c3.2)

    ENDIF( xerces_LIBRARIES )
ENDIF(xerces_INCLUDE_DIR)

IF(NOT xerces_LIBRARIES)
		MESSAGE (STATUS "${BoldYellow}libxerces-c not found${ColourReset}")
ENDIF()

