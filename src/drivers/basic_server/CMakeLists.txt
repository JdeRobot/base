


execute_process(COMMAND slice2cpp myInterface.ice --output-dir ${CMAKE_CURRENT_BINARY_DIR}
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

SET( SOURCE_FILES  basic_server.cpp myInterface.cpp)


include_directories(
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${easyiceconfig_INCLUDE_DIRS}
)

link_directories(
	${easyiceconfig_LIBRARY_DIRS}
)


add_executable (basic_server  ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(basic_server
        ${CMAKE_THREAD_LIBS_INIT}
	${ZeroCIce_LIBRARIES}  
    	${easyiceconfig_LIBRARIES}   	
)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/basic_server.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
