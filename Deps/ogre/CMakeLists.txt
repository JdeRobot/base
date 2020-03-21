PKG_CHECK_MODULES(OGRE OGRE)
#include_directories(${OGRE_INCLUDE_DIRS})

if (OGRE_VERSION)

	FOREACH(ogreInc ${OGRE_INCLUDE_DIRS})
        	include_directories(${ogreInc}/Paging)
	        include_directories(${ogreInc})
	ENDFOREACH(ogreInc)
	MESSAGE(STATUS "Ogre libraries found")
	list(APPEND DEPS libogre-1.9.0v5)
	list(APPEND DEPS_DEV libogre-1.9-dev)
else()
	MESSAGE("-- ${BoldYellow}Ogre libraries not found${ColourReset}")
endif()

