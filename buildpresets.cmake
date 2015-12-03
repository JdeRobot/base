#  Copyright (C) 2015 JDE Developers Team
#  Authors:
#    Victor Arribas <v.arribas.urjc@gmail.com>
#    Francisco Perez


## New alias for build-default
if (build_all)
	set(build-default ON)
	return()
endif()

### Notice: definition should be from large to core to allow chainning

## A run once macro to add suffix to project name ;)
macro (project_suffix suffix)
	if (NOT _project_suffix_done)
		project(${PROJECT_NAME}-${suffix})
		set(_project_suffix_done ON)
	endif()
endmacro()


## CMakeCache is a bad friend for this feature. Tweak it to avoid
function (build_component component value)
	if (NOT DEFINED build_${component})
		set(build_${component} ${value} CACHE BOOL "Build flag for JdeRobot component: ${component} (defined by builtpresets)" ${ARGS})
	elseif (NOT build_${component} EQUAL ${value})
		set(build_${component} ${value} CACHE BOOL "Build flag for JdeRobot component: ${component} (overriden by builtpresets)" FORCE ${ARGS})
	endif()
endfunction()


## Tests
if (test_ardrone)
	build_component(ardrone_server ON)
	build_component(introrob_py ON)

	build_component(core ON)
endif()

if (test_quadrotor)
	build_component(gazeboserver ON)
	build_component(quadrotor2 ON)
	build_component(quadrotor ON)
	build_component(introrob_py ON)

	build_component(core ON)
endif()

if (test_car)
	build_component(gazeboserver ON)
	build_component(car ON)
	build_component(introrob_qt ON)

	build_component(core ON)
endif()





## Build aliases (real packages for cpack)
if (build_drivers)
	project_suffix(drivers)
	set(build-default OFF)

	build_component(core ON)
	# todo: core drivers

endif()

if (build_core)
	project_suffix(core)
	set(build-default OFF)

	build_component(msgs ON)
	# libs are included by default at now
endif()

if (build_msgs)
	project_suffix(msgs)
	set(build-default OFF)

	build_component(interfaces_cpp ON)
	build_component(interfaces_java ON)
	build_component(interfaces_python ON)
	# todo: remove libs from this build flavor
endif()

#message(SEND_ERROR "PROJECT_NAME: ${PROJECT_NAME}")

