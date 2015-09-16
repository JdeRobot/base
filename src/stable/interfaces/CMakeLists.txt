include (${CMAKE_CURRENT_LIST_DIR}/slice/CMakeLists.txt)

include_directories(
        ${CMAKE_CURRENT_LIST_DIR}/cpp/jderobot
        ${CMAKE_CURRENT_LIST_DIR}/cpp/
)
add_library (JderobotInterfaces SHARED ${SOURCE_FILES} ${ICE_FILES})
TARGET_LINK_LIBRARIES(JderobotInterfaces ${ice2_LIBRARIES})
