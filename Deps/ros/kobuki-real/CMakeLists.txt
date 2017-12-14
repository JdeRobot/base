#[[find_package(catkin QUIET COMPONENTS
	      kobuki
	      kobuki_core
	    )

if(kobuki_FOUND)
 message("*** kobuki FOUND")
else()
 message("*** kobuki NOT FOUND")
endif()

if(kobuki_core_FOUND)
 message("*** kobuki_core FOUND")
else()
 message("*** kobuki_core NOT FOUND")
endif()
]]
list(APPEND DEPS ros-kinetic-kobuki ros-kinetic-kobuki-core)