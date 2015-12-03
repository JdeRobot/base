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


## Tests
if (test_quadrotor)
	set(build_quadrotor2 ON)
	set(build_introrob_py ON)

	set(build_core ON)
endif()

if (test_car)
	set(build_car ON)
	set(build_introrob_qt ON)

	set(build_core ON)
endif()


# Gazebo plugins: patch to build parent dependency
if (build_car OR build_quadrotor)
	set(build_gazeboserver ON)
endif()




## Build aliases (real packages for cpack)
if (build_drivers)
	project_suffix(drivers)
	set(build-default OFF)
	set(build_core ON)
	# core drivers

endif()

if (build_core)
	project_suffix(core)
	set(build-default OFF)
	set(build_msgs ON)
	# libs are included by default at now
endif()

if (build_msgs)
	project_suffix(msgs)
	set(build-default OFF)
	set(build_interfaces_cpp ON)
	set(build_interfaces_java ON)
	set(build_interfaces_python ON)
	# todo: remove libs from this build flavor
endif()

#message(SEND_ERROR "PROJECT_NAME: ${PROJECT_NAME}")

