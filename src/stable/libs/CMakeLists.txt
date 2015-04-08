list_subdirectories( LIST_LIBS ${CMAKE_CURRENT_SOURCE_DIR} 1)

FOREACH (libs ${LIST_LIBS})
	if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${libs}/CMakeLists.txt)
		MESSAGE(STATUS "Processing library ${libs}")
	        ADD_SUBDIRECTORY (${libs})
	endif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${libs}/CMakeLists.txt)
ENDFOREACH()
