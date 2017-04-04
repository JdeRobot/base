
## INSTALL ##

# Install .py
FILE(GLOB_RECURSE HEADERS_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*py)
FOREACH(header ${HEADERS_FILES})
	INSTALL(FILES ${header} DESTINATION share/jderobot/python/MAVLinkServer/ COMPONENT drivers)
ENDFOREACH(header)

# Install .xml
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/MAVLink.xml DESTINATION share/jderobot/python/MAVLinkServer COMPONENT drivers )

# Install mav.parm
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/mav.parm DESTINATION share/jderobot/python/MAVLinkServer COMPONENT drivers )

# Install modules
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/modules DESTINATION share/jderobot/python/MAVLinkServer COMPONENT drivers PATTERN .svn EXCLUDE)

# Install tools
INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tools DESTINATION share/jderobot/python/MAVLinkServer COMPONENT drivers PATTERN .svn EXCLUDE)
