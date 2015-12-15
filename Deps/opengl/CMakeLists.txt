include(FindPkgConfig)
find_package(GLUT)
find_package(OpenGL REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS})

link_directories(${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )


IF (GLUT_LIBRARIES) 
	MESSAGE("OpenGL libraries found at ${GLUT_LIBRARY}")
	list(APPEND DEPS freeglut3 libglu1-mesa)
	list(APPEND DEPS_DEV freeglut3-dev libglu1-mesa-dev)
ELSE()
	MESSAGE ("*** OpenGL not found")
ENDIF()

