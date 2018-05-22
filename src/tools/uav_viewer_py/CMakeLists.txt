
configure_file(
    uav_viewer_py.in
    uav_viewer_py
    @ONLY
)

## INSTALL ##

# Install launcher
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/uav_viewer_py
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
    DESTINATION bin
    COMPONENT uav-viewer-python
)

# Install .py
FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*py)
FOREACH(header ${HEADERS_FILES})
	INSTALL(FILES ${header} DESTINATION share/jderobot/python/uav_viewer_py/ COMPONENT uav-viewer-python)
ENDFOREACH(header)

# Install gui
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui DESTINATION share/jderobot/python/uav_viewer_py COMPONENT uav-viewer-python PATTERN .svn EXCLUDE)


INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/uav_viewer_py.yml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT uav-viewer-python)
