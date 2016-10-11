
## INSTALL ##

# Install main.py
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/main.py DESTINATION share/jderobot/python/basic_component_py COMPONENT python3 )

# Install gui
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui DESTINATION share/jderobot/python/basic_component_py COMPONENT python3 PATTERN .svn EXCLUDE)

# Install resources
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources DESTINATION share/jderobot/python/basic_component_py COMPONENT python3 PATTERN .svn EXCLUDE)
