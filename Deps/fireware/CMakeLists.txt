# CHECK FIREWARE 

FIND_PATH( raw_INCLUDE_DIR NAMES raw1394.h  PATHS ENV C++LIB ENV PATH PATH_SUFFIXES libraw1394)
FIND_PATH(dc_INCLUDE_DIR  NAMES dc1394.h  PATHS ENV C++LIB ENV PATH PATH_SUFFIXES dc1394)

FIND_LIBRARY( with_fireware NAMES dc1394 raw1394 PATHS ENV C++LIB ENV PATH PATH_SUFFIXES lib lib64 /usr/lib)

IF(NOT with_fireware)
    MESSAGE ("-- ${BoldYellow}Fireware libraries: dc1394 or raw1394 not found${ColourReset}") 

ENDIF()

IF(with_fireware)
    MESSAGE ("-- Fireware found") 
    include_directories(/usr/include)
    link_directories(/usr/lib)
    list(APPEND DEPS libdc1394-22)
    list(APPEND DEPS_DEV libdc1394-22-dev)
ENDIF(with_fireware)



