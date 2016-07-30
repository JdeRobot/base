cmake_minimum_required(VERSION 2.8)

configure_file(
    easyiceconfig/hardcodedpaths.py.in
    easyiceconfig/hardcodedpaths.py
    @ONLY
)

file(GLOB ModuleFiles 
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/easyiceconfig
    ${CMAKE_CURRENT_SOURCE_DIR}/easyiceconfig/*)

string(REGEX REPLACE "[^;]+\\.in;?" "" ModuleFiles "${ModuleFiles}")


add_custom_target(easyiceconfig_py)
foreach(ModuleFile ${ModuleFiles})
    add_custom_command(TARGET easyiceconfig_py 
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/easyiceconfig/${ModuleFile} ${CMAKE_CURRENT_BINARY_DIR}/easyiceconfig/${ModuleFile}
    )
endforeach(ModuleFile ${ModuleFiles})

### Install
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/easyiceconfig
    DESTINATION ${PYTHON_MODULE_PATH}
    COMPONENT core
)
