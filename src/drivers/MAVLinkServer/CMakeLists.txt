
## INSTALL ##

# Install .py
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.py DESTINATION share/jderobot/python/MAVLinkServer COMPONENT python3 )

# Install .xml
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/MAVLink.xml DESTINATION share/jderobot/python/MAVLinkServer COMPONENT python3 )

# Install mav.parm
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/mav.parm DESTINATION share/jderobot/python/MAVLinkServer COMPONENT python3 )

# Install modules
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/modules DESTINATION share/jderobot/python/MAVLinkServer COMPONENT python3 PATTERN .svn EXCLUDE)

# Install tools
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tools DESTINATION share/jderobot/python/MAVLinkServer COMPONENT python3 PATTERN .svn EXCLUDE)
