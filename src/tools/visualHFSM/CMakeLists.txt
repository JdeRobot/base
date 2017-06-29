IF (goocanvasmm_INCLUDE_DIRS)

SET( SOURCE_FILES_VISUALHFSM
	iceinterface.cpp
	point.cpp
	node.cpp
	transition.cpp
	guinode.cpp
	subautomata.cpp
	guitransition.cpp
	guisubautomata.cpp
	generate.cpp
	popups/configfiledialog.cpp
	popups/editnodedialog.cpp
	popups/edittransitiondialog.cpp
    popups/edittransitioncodedialog.cpp
	popups/funvardialog.cpp
	popups/librariesdialog.cpp
	popups/loadfiledialog.cpp
	popups/renamedialog.cpp
	popups/renametransitiondialog.cpp
	popups/savefiledialog.cpp
	popups/timerdialog.cpp
	savefile.cpp
	xmlparser.cpp
	visualhfsm.cpp
	main.cpp
)


SET( SOURCE_FILES_RUNTIMEGUI
		automatagui.cpp
		point.cpp
		node.cpp
		transition.cpp
		subautomata.cpp
		guinode.cpp
		guitransition.cpp
		guisubautomata.cpp
		popups/editnodedialog.cpp
		popups/edittransitiondialog.cpp
		popups/edittransitioncodedialog.cpp
		popups/renamedialog.cpp
		popups/renametransitiondialog.cpp
)

link_directories(
	${resourcelocator_LIBRARY_DIRS}
)

include_directories (
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${gtkmm3_INCLUDE_DIRS}
    ${goocanvasmm_INCLUDE_DIRS}
    ${gtk3_INCLUDE_DIRS}
    ${libxmlpp_INCLUDE_DIRS}
    ${resourcelocator_INCLUDE_DIRS}
)

add_executable (visualHFSM ${SOURCE_FILES_VISUALHFSM})
add_library(visualHFSMlib ${SOURCE_FILES_RUNTIMEGUI})

TARGET_LINK_LIBRARIES ( visualHFSM 
    ${gtkmm3_LIBRARIES}
    ${goocanvasmm_LIBRARIES}
    ${gtk3_LIBRARIES}
    ${libxmlpp_LIBRARIES}
    ${resourcelocator_LIBRARIES}
		${GLOG_LIBRARIES}

		)

TARGET_LINK_LIBRARIES ( visualHFSMlib
		${gtkmm3_LIBRARIES}
		${goocanvasmm_LIBRARIES}
		${gtk3_LIBRARIES}
		${libxmlpp_LIBRARIES}
		${resourcelocator_LIBRARIES}
		${GLOG_LIBRARIES}
		)

add_custom_command(TARGET visualHFSM POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/gui $<TARGET_FILE_DIR:visualHFSM>/gui)

add_custom_command(TARGET visualHFSM POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/getinterfaces_new.sh $<TARGET_FILE_DIR:visualHFSM>)

    

    install(TARGETS visualHFSM
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
    COMPONENT tools
    )

	install(TARGETS visualHFSMlib DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/jderobot/ COMPONENT tools)

    # Install Glade
	FILE(GLOB_RECURSE GLADE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/gui/*.glade)
	INSTALL (FILES ${GLADE_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/glade)

	# install resources (jderobot logo)
	INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/jderobot.png DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/resources)

ENDIF()




