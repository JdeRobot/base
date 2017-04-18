
SET( SOURCE_FILES namingServiceApp.cpp NamingServiceJdeRobot.h NamingServiceJdeRobot.cpp)

add_definitions(-DGLADE_DIR="${gladedir}")

include_directories(
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}   
    ${easyiceconfig_INCLUDE_DIRS} 
)

link_directories(
	${easyiceconfig_LIBRARY_DIRS}
)

add_executable (namingService ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(namingService
    ${CMAKE_THREAD_LIBS_INIT}
    ${gsl_LIBRARIES}    
    ${ZeroCIce_LIBRARIES}
    ${Boost_LIBRARIES}  
    ${easyiceconfig_LIBRARIES}   
    JderobotInterfaces
    logger
	${GLOG_LIBRARIES}
)

INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/namingService.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
