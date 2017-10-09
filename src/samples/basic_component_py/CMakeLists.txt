
configure_file(
    basic_component_py.in
    basic_component_py
    @ONLY
)

## INSTALL ##

# install Launcher
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/basic_component_py
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
    DESTINATION bin
    COMPONENT basic-component-python
)

# Install .py
FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*py)
FOREACH(header ${HEADERS_FILES})
	INSTALL(FILES ${header} DESTINATION share/jderobot/python/basic_component_py/ COMPONENT basic-component-python)
ENDFOREACH(header)

# Install gui
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui DESTINATION share/jderobot/python/basic_component_py COMPONENT basic-component-python PATTERN .svn EXCLUDE)

# Install resources
#INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources DESTINATION share/jderobot/python/basic_component_py COMPONENT basic-component-python PATTERN .svn EXCLUDE)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/basic_component_py.yml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf
COMPONENT basic-component-python)
