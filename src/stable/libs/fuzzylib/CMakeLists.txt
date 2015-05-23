include_directories(${CMAKE_CURRENT_SOURCE_DIR})
SET(fuzzylib_CPP fuzzylib.c)
SET(fuzzylib_H fuzzylib.h)

add_library (fuzzylib SHARED ${fuzzylib_CPP} ${fuzzylib_H})


INSTALL (TARGETS fuzzylib DESTINATION lib/jderobot)
INSTALL (FILES ${fuzzylib_H} DESTINATION include/jderobot/fuzzylib/)
