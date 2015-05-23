SET(jderobotutilshare_CPP observer.cpp paramdict.cpp time.cpp)
SET(jderobotutilshare_H jderobotutil.h observer.h paramdict.h time.h uncopyable.h)

add_library (jderobotutil SHARED ${jderobotutilshare_CPP} ${jderobotutilshare_H})

INSTALL (TARGETS jderobotutil DESTINATION lib/jderobot)
INSTALL (FILES ${jderobotutil_H} DESTINATION include/jderobot/jderobotutil/)
