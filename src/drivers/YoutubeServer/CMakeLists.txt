configure_file(
    youtubeserver.in
    youtubeserver
    @ONLY
)
## INSTALL ##

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/youtubeserver
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    COMPONENT youtubeserver-python
)

# Install .py
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/videoTools DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/python/youtubeserver/ COMPONENT youtubeserver-python)
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/JdeRobot DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/python/youtubeserver/ COMPONENT youtubeserver-python)
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/youtubeserver.py DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/python/youtubeserver/ COMPONENT youtubeserver-python)


INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/youtubeserver.yml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT youtubeserver-python)
