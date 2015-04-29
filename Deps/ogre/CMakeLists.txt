PKG_CHECK_MODULES(OGRE OGRE )
#include_directories(${OGRE_INCLUDE_DIRS})
link_directories(${OGRE_LIBRARY_DIRS})


if (OGRE_LIBRARY_DIRS)

	FOREACH(ogreInc ${OGRE_INCLUDE_DIRS})
        	include_directories(${ogreInc}/Paging)
	        include_directories(${ogreInc})
	ENDFOREACH(ogreInc)
	MESSAGE("*** OGRe LIBRARIES FOUND: ${OGRE_INCLUDE_DIRS}")
	SET(DEPS "${DEPS} libogre-1.8.0, libogre-1.8-dev")
else()
	MESSAGE("*** OGRE LIBRARIES NOT FOUND")
endif()

