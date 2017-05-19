configure_file(
    colorTuner.in
    colorTuner
    @ONLY
)
## INSTALL ##

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/colorTuner
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
    DESTINATION bin
)

INSTALL(FILES colorTuner.py DESTINATION share/jderobot/python/colorTuner_py/ COMPONENT tools)
INSTALL(FILES resources_rc.py DESTINATION share/jderobot/python/colorTuner_py/ COMPONENT tools)

# Install gui
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui DESTINATION share/jderobot/python/colorTuner_py COMPONENT tools PATTERN .svn EXCLUDE)

# Install sensors
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sensors DESTINATION share/jderobot/python/colorTuner_py COMPONENT tools PATTERN .svn EXCLUDE)

# Install Filters
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/filters DESTINATION share/jderobot/python/colorTuner_py COMPONENT tools PATTERN .svn EXCLUDE)


INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/colorTuner_py.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
