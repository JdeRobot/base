
cmake_minimum_required(VERSION 2.8)

project(evicam_driver)

set (SRC
     src/evicam_driver.cpp
     src/interfaces/pantilti.cpp
     EVILib/EVI-D100g.c++
     EVILib/EVI-D100P.c++
     EVILib/EVILib.c++)

include_directories(${INTERFACES_CPP_DIR}
        ${LIBS_DIR}/
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${easyiceconfig_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/EVILib
)

link_directories(
        ${easyiceconfig_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME} ${SRC})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    ${ZeroCIce_LIBRARIES}
    JderobotInterfaces
    ${easyiceconfig_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)


add_executable(testpt src/test.cpp)
TARGET_LINK_LIBRARIES(testpt
    ${ZeroCIce_LIBRARIES}
    JderobotInterfaces
    ${easyiceconfig_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

install(TARGETS ${PROJECT_NAME}
          DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
          COMPONENT drivers
        )

INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/evicam_driver.cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf)
