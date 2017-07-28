configure_file(
    cameraview_py.in
    cameraview_py
    @ONLY
)

## INSTALL ##

# install Launcher
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/cameraview_py
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
    DESTINATION bin
)

# Install .py
FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*py)
FOREACH(header ${HEADERS_FILES})
	INSTALL(FILES ${header} DESTINATION share/jderobot/python/cameraview_py/ COMPONENT tools)
ENDFOREACH(header)

# Install resources
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/cameraview_py.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
