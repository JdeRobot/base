SET(pioneer_CPP pioneer.c)
SET(pioneer_H pioneer.h)

add_library (pioneer SHARED ${pioneer_CPP} ${pioneer_H})
INSTALL (TARGETS pioneer DESTINATION lib/jderobot)
INSTALL (FILES ${pioneer_H} DESTINATION include/jderobot/pioneer/)
