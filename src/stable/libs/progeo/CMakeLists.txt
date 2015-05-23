SET(progeo_CPP progeo.c)
SET(progeo_H progeo.h)

add_library (progeo SHARED ${progeo_CPP} ${progeo_H})

TARGET_LINK_LIBRARIES(progeo ${gsl_LIBRARIES} ${xerces_LIBRARIES} ${LIBXML2_LIBRARIES})

INSTALL (TARGETS progeo DESTINATION lib/jderobot)
INSTALL (FILES ${progeo_H} DESTINATION include/jderobot/progeo/)
