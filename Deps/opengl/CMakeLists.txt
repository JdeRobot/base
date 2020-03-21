include(FindPkgConfig)
find_package(GLUT)
find_package(OpenGL REQUIRED)

IF (NOT GLUT_LIBRARIES MATCHES ".*NOTFOUND.*" AND
    NOT OPENGL_LIBRARIES MATCHES ".*NOTFOUND.*") 
        MESSAGE(STATUS "OpenGL libraries found")

        include_directories(${GLUT_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS})
        link_directories(${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} )

        list(APPEND DEPS freeglut3 libglu1-mesa)
        list(APPEND DEPS_DEV freeglut3-dev libglu1-mesa-dev)
ELSE()
        MESSAGE (STATUS "${BoldYellow}OpenGL not found${ColourReset}")
ENDIF()


