set(SLICE_NEW_STYLE "ON")
if (NOT DEFINED SLICE_NEW_STYLE)

include (${CMAKE_CURRENT_LIST_DIR}/slice/CMakeLists.txt)

include_directories(
        ${CMAKE_CURRENT_LIST_DIR}/cpp/jderobot
        ${CMAKE_CURRENT_LIST_DIR}/cpp/
)
add_library (JderobotInterfaces SHARED ${SOURCE_FILES} ${ICE_FILES})
TARGET_LINK_LIBRARIES(JderobotInterfaces ${ice2_LIBRARIES})

else()


set(SLICE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/slice)
set(SLICE_CPP_DIR ${CMAKE_CURRENT_BINARY_DIR}/cpp)
set(SLICE_PY_DIR ${CMAKE_CURRENT_BINARY_DIR}/python)
set(SLICE_JAVA_DIR ${CMAKE_CURRENT_BINARY_DIR}/java)

FILE(GLOB_RECURSE SLICE_FILES
    RELATIVE ${SLICE_DIRECTORY}
    "*.ice"
)

#message(STATUS "SLICE_FILES ${SLICE_FILES}")
#message(STATUS "CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}")

message(STATUS "Building SLICE Interfaces for CPP")


foreach(slice_rel_path ${SLICE_FILES})
    get_filename_component(slice_name ${slice_rel_path} NAME_WE)
    get_filename_component(slice_file_name ${slice_rel_path} NAME)
    get_filename_component(slice_rel_dir   ${slice_rel_path} PATH)
    set(slice_abs_path  ${SLICE_DIRECTORY}/${slice_rel_path})
    set(slice_abs_dir  ${SLICE_DIRECTORY}/${slice_rel_dir})

#    message(STATUS "slice_name ${slice_name}")
#    message(STATUS "slice_file_name ${slice_file_name}")
#    message(STATUS "slice_rel_dir ${slice_rel_dir}")
#    message(STATUS "slice_abs_path ${slice_abs_path}")
#    message(STATUS "slice_abs_dir ${slice_abs_dir}")

    ### CPP
    set(target_directory ${SLICE_CPP_DIR}/${slice_rel_dir})
    file(MAKE_DIRECTORY ${target_directory})

if(DEFINED SLICE_ON_MAKE)
    add_custom_command(
        OUTPUT ${target_directory}/${slice_name}.h ${target_directory}/${slice_name}.cpp
        WORKING_DIRECTORY ${target_directory}
        COMMAND slice2cpp ${slice_file_name} -I${SLICE_DIRECTORY} -I${SLICE_CPP_DIR} --output-dir ${target_directory} --include-dir ${slice_rel_dir}
        COMMENT "Ice: Processing ${slice_name} for CPP"
    )
else() # SLICE_ON_CMAKE
    execute_process(
        WORKING_DIRECTORY ${target_directory}
        COMMAND slice2cpp ${slice_abs_path} -I${SLICE_DIRECTORY} --output-dir ${target_directory} --include-dir ${slice_rel_dir}
    )
endif()
    #LIST(APPEND SLICE_CPP_GENERATED ${target_directory}/${slice_name}.h)
    LIST(APPEND SLICE_CPP_GENERATED ${target_directory}/${slice_name}.cpp)


    ### Python
    set(target_directory ${SLICE_PY_DIR}/${slice_rel_dir})
    file(MAKE_DIRECTORY ${target_directory})
    execute_process(
        WORKING_DIRECTORY ${SLICE_PY_DIR}
        COMMAND slice2py ${slice_abs_path} -I${SLICE_DIRECTORY} --output-dir ${SLICE_PY_DIR}
    )

    ### Java
    set(target_directory ${SLICE_JAVA_DIR}/${slice_rel_dir})
    file(MAKE_DIRECTORY ${target_directory})
    execute_process(
        WORKING_DIRECTORY ${SLICE_JAVA_DIR}
        COMMAND slice2java ${slice_abs_path} -I${SLICE_DIRECTORY} --output-dir ${SLICE_JAVA_DIR}
    )

endforeach()


unset(slice_abs_dir)
unset(slice_abs_path)
unset(slice_rel_dir)
unset(slice_file_name)
unset(slice_name)
unset(target_directory)
unset(SLICE_DIRECTORY)


### CPP
include_directories(${SLICE_CPP_DIR})
add_library (JderobotInterfaces SHARED ${SLICE_CPP_GENERATED})
target_link_libraries(JderobotInterfaces ${ice2_LIBRARIES})

endif(NOT DEFINED SLICE_NEW_STYLE)
