
SET(Logger_CPP Logger.cpp)
SET(Logger_H Logger.h)

ADD_LIBRARY(logger SHARED ${Logger_CPP} ${Logger_H})

target_link_libraries(logger ${Boost_LIBRARIES})

SET_PROPERTY(TARGET logger PROPERTY SOVERSION 0.1.0)

INSTALL (TARGETS logger DESTINATION lib/jderobot)
INSTALL (FILES ${logger_H} DESTINATION include/jderobot/logger/)
 
