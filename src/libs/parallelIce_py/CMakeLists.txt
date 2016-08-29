cmake_minimum_required(VERSION 2.8)


file(GLOB ModuleFiles 

    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/parallelIce

    ${CMAKE_CURRENT_SOURCE_DIR}/parallelIce/*)


add_custom_target(parallelIce_py)

foreach(ModuleFile ${ModuleFiles})

    add_custom_command(TARGET parallelIce_py 

        COMMAND ${CMAKE_COMMAND} -E copy

        ${CMAKE_CURRENT_SOURCE_DIR}/parallelIce/${ModuleFile} ${CMAKE_CURRENT_BINARY_DIR}/parallelIce/${ModuleFile}

    )

endforeach(ModuleFile ${ModuleFiles})



### Install

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/parallelIce

    DESTINATION ${PYTHON_MODULE_PATH}

    COMPONENT core

)