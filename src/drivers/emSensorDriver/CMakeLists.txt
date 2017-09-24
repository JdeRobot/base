
find_library(wiringPi_LIBRARY wiringPi)

if(wiringPi_LIBRARY)

    message("WiringPi library found: ${wiringPi_LIBRARY}")
    if(${Boost_THREAD_FOUND})

        include_directories(
            ${INTERFACES_CPP_DIR}
            ${LIBS_DIR}
            ${CMAKE_CURRENT_BINARY_DIR}
            ${easyiceconfig_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIR}
        )

        link_directories(
            ${easyiceconfig_LIBRARY_DIRS}
        )
    
        add_executable (emSensorDriver
              emSensorDriver.cpp
              sharer.cpp
              pwm_analyzer.cpp
              gpio_reader.cpp
              emSensorI.cpp
        )

        target_link_libraries(emSensorDriver
                ${CMAKE_THREAD_LIBS_INIT}
                JderobotInterfaces
                ${easyiceconfig_LIBRARIES}
                ${ZeroCIce_LIBRARIES}
                ${Boost_LIBRARIES}
                ${wiringPi_LIBRARY}
        )

        INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/emSensorDriver.cfg 
          DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf
          COMPONENT emSensorDriver
        )

        INSTALL(TARGETS emSensorDriver
          DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
          COMPONENT emSensorDriver
        )

    else()
      message("Boost thread library is required to build emSensorDriver, skipping driver build.")
    endif()
else()
      message("WiringPi library is required to build emSensorDriver, skipping driver bulid.")
endif()
