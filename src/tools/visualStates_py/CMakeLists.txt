cmake_minimum_required(VERSION 2.8)

SET(SOURCE_CPP_RUNTIME
	codegen/cpp/state.cpp
	codegen/cpp/transition.cpp
	codegen/cpp/temporaltransition.cpp
    codegen/cpp/runtimegui.cpp
)

INCLUDE_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/include)
LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib)

ADD_LIBRARY(visualStatesRunTime SHARED ${SOURCE_CPP_RUNTIME})
TARGET_LINK_LIBRARIES(visualStatesRunTime easyiceconfig Ice JderobotInterfaces IceUtil pthread)

INSTALL(TARGETS visualStatesRunTime DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/visualstates_py/)

#install runtime headers
INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/codegen/cpp/
		DESTINATION ${CMAKE_INSTALL_PREFIX}/include/visualstates_py/
		FILES_MATCHING PATTERN "*.h")

#install python runtime dependencies
INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/codegen/python/
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py/codegen/python/
		PATTERN __pycache__ EXCLUDE )

# install python runtime gui dependencies
INSTALL(FILES ${CMAKE_SOURCE_DIR}/gui/state.py
		${CMAKE_SOURCE_DIR}/gui/transition.py
		${CMAKE_SOURCE_DIR}/gui/guitransition.py
		${CMAKE_SOURCE_DIR}/gui/guistate.py
		${CMAKE_SOURCE_DIR}/gui/idtextboxgraphicsitem.py
		${CMAKE_SOURCE_DIR}/gui/recthandlegraphicsitem.py
		${CMAKE_SOURCE_DIR}/gui/transitiontype.py
		${CMAKE_SOURCE_DIR}/gui/treemodel.py
		${CMAKE_SOURCE_DIR}/gui/treenode.py
		${CMAKE_SOURCE_DIR}/gui/cmakevars.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py/gui/)

# install python module __init__.py
INSTALL(FILES ${CMAKE_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py/)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py/codegen/)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py/codegen/python/)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/visualStates_py/gui/)

# install getinterfaces.sh bash script to be able to get the interfaces
INSTALL(FILES ${CMAKE_SOURCE_DIR}/getinterfaces.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/ PERMISSIONS
OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

# set the cmake variables to be used in the source code
configure_file(${CMAKE_SOURCE_DIR}/gui/cmakevars.py.in ${CMAKE_SOURCE_DIR}/gui/cmakevars.py)
