PKG_CHECK_MODULES(GAZEBO QUIET gazebo<=5.1.0)
link_directories(${GAZEBO_LIBRARY_DIRS})


if (GAZEBO_INCLUDE_DIRS)

	FOREACH(gazeboInc ${GAZEBO_INCLUDE_DIRS})
        	include_directories(${gazeboInc}/gazebo/)
	        include_directories(${gazeboInc})
	ENDFOREACH(gazeboInc)
	MESSAGE("*** GAZEBO LIBRARIES FOUND: ${GAZEBO_INCLUDE_DIRS} - ${OGRE_INCLUDE_DIRS}")
	list(APPEND DEPS gazebo5)
	list(APPEND DEPS_DEV libgazebo5-dev)
else()
	MESSAGE("*** GAZEBO LIBRARIES NOT FOUND")
endif()

