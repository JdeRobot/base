configure_file(
    youtubeserver.in
    youtubeserver
    @ONLY
)
## INSTALL ##

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/youtubeserver
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
    DESTINATION bin
    COMPONENT youtubeserver-python
)

# Install .py
FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*py)
FOREACH(header ${HEADERS_FILES})
	INSTALL(FILES ${header} DESTINATION share/jderobot/python/youtubeserver/ COMPONENT youtubeserver-python)
ENDFOREACH(header)


INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/youtubeserver.yml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT youtubeserver-python)
