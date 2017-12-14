
#[[find_package(catkin QUIET COMPONENTS
	      kobuki
	      kobuki_core
	    )

messge(kobuki_FOUND)
if(kobuki_FOUND)
 message("*** kobuki FOUND")
else(kobuki_FOUND)
 message("*** kobuki NOT FOUND")
endif(kobuki_FOUND)

if(kobuki_core_FOUND)
 message("*** kobuki_core FOUND")
else(kobuki_core_FOUND)
 message("*** kobuki_core NOT FOUND")
endif(kobuki_core_FOUND)
]]
list(APPEND DEPS ros-kinetic-kobuki ros-kinetic-kobuki-core)