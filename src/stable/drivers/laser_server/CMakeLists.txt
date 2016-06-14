set (NAME laser_server)
cmake_minimum_required(VERSION 2.8)
set (SRC
     main.cpp
     hokuyo/hokuyo.cpp
     hokuyo/hokuyomanager.cpp
     laseri.cpp)

include_directories(${INTERFACES_CPP_DIR}
        ${LIBS_DIR}/
        ${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/hokuyo)

add_executable(${NAME} ${SRC})
TARGET_LINK_LIBRARIES(${NAME}
    ${ZeroCIce_LIBRARIES}
    JderobotInterfaces)
