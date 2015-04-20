PKG_CHECK_MODULES(OGRE OGRE )
#include_directories(${OGRE_INCLUDE_DIRS})
link_directories(${OGRE_LIBRARY_DIRS})


if (GAZEBO_INCLUDE_DIRS)

	FOREACH(ogreInc ${OGRE_INCLUDE_DIRS})
        	include_directories(${ogreInc}/Paging)
	        include_directories(${ogreInc})
	ENDFOREACH(ogreInc)
	MESSAGE("*** OGRe LIBRARIES FOUND: ${OGRE_INCLUDE_DIRS}")
	SET(DEPS "${DEPS} ogre,")
else()
	MESSAGE("*** OGRE LIBRARIES NOT FOUND")
endif()

