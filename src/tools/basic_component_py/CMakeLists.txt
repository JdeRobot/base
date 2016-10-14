
## INSTALL ##

# Install .py
FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*py)
FOREACH(header ${HEADERS_FILES})
	INSTALL(FILES ${header} DESTINATION share/jderobot/python/basic_component_py/ COMPONENT tools)
ENDFOREACH(header)

# Install gui
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui DESTINATION share/jderobot/python/basic_component_py COMPONENT tools PATTERN .svn EXCLUDE)

# Install resources
#INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources DESTINATION share/jderobot/python/basic_component_py COMPONENT tools PATTERN .svn EXCLUDE)
