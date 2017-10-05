set (LIST_LIBS "config_cpp") #Adding library to be processed first
list_subdirectories3( LIST_LIBS ${CMAKE_CURRENT_SOURCE_DIR} 1)

FOREACH (lib ${LIST_LIBS})
	if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${lib}/CMakeLists.txt)
		option(build_${lib} "Build library ${lib}" ON)
		if (build_${lib})
			MESSAGE(STATUS "Processing library ${lib}")
			ADD_SUBDIRECTORY (${lib})
		endif()
	endif()
ENDFOREACH()
