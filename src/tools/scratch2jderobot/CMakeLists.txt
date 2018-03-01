IF (ENABLE_ROS)
    project(scratch2jderobot)
	## Find catkin macros and libraries
	find_package(catkin REQUIRED COMPONENTS
	    rospy
	)

	## Uncomment this if the package has a setup.py. This macro ensures
	## modules and global scripts declared therein get installed
	## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
	catkin_python_setup()

	################################################
	## Declare ROS messages, services and actions ##
	################################################

	## Generate messages in the 'msg' folder
	#add_message_files(
	    #DIRECTORY msg
	    #FILES
	    #Example.msg
	#)

	## Generate services in the 'srv' folder
	#add_service_files(
	    #DIRECTORY srv
	    #FILES
	    #Example.srv
	#)

	## Generate added messages and services with any dependencies listed here
	#generate_messages()

	###################################
	## catkin specific configuration ##
	###################################

	## Declare things to be passed to dependent projects
	catkin_package(
	    INCLUDE_DIRS src
	    CATKIN_DEPENDS rospy
	    DEPENDS 
	)

	###########
	## Build ##
	###########

	## Specify additional locations of header files
	## Your package locations should be listed before other locations
	include_directories(
	    include src
	    ${catkin_INCLUDE_DIRS}
	)

	#############
	## Testing ##
	#############

	## Add folders to be run by python nosetests
	#catkin_add_nosetests(src/node/test.py)

	#if(CATKIN_ENABLE_TESTING)
	#    find_package(rostest REQUIRED)
	#    add_rostest(test/.test)
	#endif()

	#############
	## INSTALL ##
	#############


	#INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/cfg/robot.yml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT scratch2jderobot)

	#INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/cfg/drone.yml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT scratch2jderobot)

	INSTALL (DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/src DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/python/scratch2jderobot/ COMPONENT scratch2jderobot)
	INSTALL (DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/python/scratch2jderobot/ COMPONENT scratch2jderobot)
	INSTALL (DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/extension DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/python/scratch2jderobot/ COMPONENT scratch2jderobot)
	INSTALL (DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/kurt DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/python/scratch2jderobot/ COMPONENT scratch2jderobot)
	INSTALL (DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/scripts DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/python/scratch2jderobot/ COMPONENT scratch2jderobot)
	INSTALL (DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/python/scratch2jderobot/ COMPONENT scratch2jderobot)
	#INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/scratch2python.py DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/python/scratch2jderobot/ COMPONENT scratch2jderobot)

ENDIF()
