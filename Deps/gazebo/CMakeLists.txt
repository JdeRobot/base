PKG_CHECK_MODULES(GAZEBO QUIET gazebo>=7.0.0)
link_directories(${GAZEBO_LIBRARY_DIRS})


if (GAZEBO_INCLUDE_DIRS)

	FOREACH(gazeboInc ${GAZEBO_INCLUDE_DIRS})
        	include_directories(${gazeboInc}/gazebo/)
	        include_directories(${gazeboInc})
	ENDFOREACH(gazeboInc)
	MESSAGE("-- Gazebo libraries found")
	list(APPEND DEPS gazebo9)
	list(APPEND DEPS_DEV libgazebo9-dev)
else()
	MESSAGE("-- ${BoldYellow}Gazebo libraries NOT found${ColourReset}")
endif()

