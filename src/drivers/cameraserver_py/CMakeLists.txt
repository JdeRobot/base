configure_file(
    cameraserver_py.in
    cameraserver_py
    @ONLY
)
## INSTALL ##

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/cameraserver_py
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
    DESTINATION bin
    COMPONENT cameraserver-python
)


INSTALL(FILES *.py DESTINATION share/jderobot/python/cameraserver_py/ COMPONENT cameraserver-python)


INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/cameraserver_py.yml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT cameraserver-python)
