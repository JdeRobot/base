cmake_minimum_required(VERSION 2.8)

### Install

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/parallelIce

    DESTINATION ${PYTHON_MODULE_PATH}

    COMPONENT core

)