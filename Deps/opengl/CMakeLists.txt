include(FindPkgConfig)
find_package(GLUT)
find_package(OpenGL REQUIRED)

IF (NOT GLUT_LIBRARIES MATCHES ".*NOTFOUND.*" AND
    NOT OPENGL_LIBRARIES MATCHES ".*NOTFOUND.*") 
        MESSAGE("OpenGL libraries found at ${GLUT_LIBRARIES}")

        include_directories(${GLUT_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS})
        link_directories(${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} )

        list(APPEND DEPS freeglut3 libglu1-mesa)
        list(APPEND DEPS_DEV freeglut3-dev libglu1-mesa-dev)
ELSE()
        MESSAGE ("*** OpenGL not found")
ENDIF()


