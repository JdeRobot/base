#add_custom_command(TARGET visualStates_py POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/gui $<TARGET_FILE_DIR:visualStates>/gui)
#
#
#    install(TARGETS visualStates
#    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
#    COMPONENT tools
#    )
#
#	install(TARGETS visualStateslib DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/ COMPONENT tools)
#
#	install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/getinterfaces.sh
#      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
#	  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
#
#	INSTALL (FILES ${GLADE_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/glade/visualStates)
#
#    # Install Glade
#	FILE(GLOB_RECURSE GLADE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/gui/*.glade)
#	INSTALL (FILES ${GLADE_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/glade/visualStates)
#
#	# install resources (jderobot logo)
#	INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/jderobot.png DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/resources)
#
#
#	# Install C++ Headers
#	FILE(GLOB h_files "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
#	INSTALL(FILES ${h_files} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/visualStates)
#
#	FILE(GLOB h_files "${CMAKE_CURRENT_SOURCE_DIR}/popups/*.h")
#	INSTALL(FILES ${h_files} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jderobot/visualStates/popups)


	# Install Python Files

	INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/gui" DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/python3/visualStates_py)




