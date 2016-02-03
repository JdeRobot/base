list_subdirectories( LIST_TOOLS ${CMAKE_CURRENT_SOURCE_DIR} 1)

IF(NOT DEFINED build-default)
	SET(build-default ON)
ENDIF(NOT DEFINED build-default)

FOREACH (tool ${LIST_TOOLS})
	if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${tool}/CMakeLists.txt)
		option(build_${tool} "Build tool ${tool}" ${build-default})
		IF(build_${tool})
			MESSAGE(STATUS "Processing tool ${tool}")
			ADD_SUBDIRECTORY (${tool})
		ENDIF(build_${tool})
	endif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${tool}/CMakeLists.txt)
ENDFOREACH()

