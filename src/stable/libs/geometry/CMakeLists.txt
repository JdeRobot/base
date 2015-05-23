PROJECT(libgeometry)
 

SET(geometry_CPP math/matriz3x3.cpp  math/matriz4x4.cpp  math/plano.cpp  math/utils.cpp math/Line2D.cpp math/Segment2D.cpp math/Point2D.cpp math/Point3D.cpp math/Line3D.cpp math/Plane3D.cpp math/Segment3D.cpp collada/colladaparser.cpp collada/color.cpp collada/malla.cpp collada/material.cpp collada/submalla.cpp progeo/Progeo.cpp math/segmento.cpp math/recta.cpp)
SET(geometry_H math/matriz3x3.h  math/matriz4x4.h  math/plano.h  math/utils.h math/Line2D.h math/Segment2D.h math/Point2D.h math/Point3D.h math/Line3D.h math/Plane3D.h math/Segment3D.h collada/colladaparser.h collada/color.h collada/malla.h collada/material.h collada/submalla.h progeo/Progeo.h math/segmento.h math/recta.h)

 
ADD_LIBRARY(geometry SHARED ${geometry_H} ${geometry_CPP})
target_link_libraries(geometry tinyxml GLU opencv_core opencv_highgui ${Boost_LIBRARIES})

SET_PROPERTY(TARGET geometry PROPERTY SOVERSION 1.0)

INSTALL (TARGETS geometry DESTINATION lib/jderobot)
INSTALL (FILES ${geometry_H} DESTINATION include/jderobot/geometry/)
