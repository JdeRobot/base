#macro to switch pythonInterp
macro(usePython ver)
  if (${ver} EQUAL 2 AND ${PYTHON2INTERP_FOUND})
    set(PYTHON_EXECUTABLE ${PYTHON2_EXECUTABLE})
    set(PYTHON_VERSION_STRING ${PYTHON2_VERSION_STRING})
    set(PYTHON_VERSION_MAJOR ${PYTHON2_VERSION_MAJOR})
    set(PYTHON_VERSION_MINOR ${PYTHON2_VERSION_MINOR})
    set(PYTHON_VERSION_PATCH ${PYTHON2_VERSION_PATCH})
    set(PYTHON_MODULE_PATH ${PYTHON2_MODULE_PATH})
    set(PYTHONINTERP_FOUND TRUE)
  elseif(${ver} EQUAL 3 AND ${PYTHON3INTERP_FOUND})
    set(PYTHON_EXECUTABLE ${PYTHON3_EXECUTABLE})
    set(PYTHON_VERSION_STRING ${PYTHON3_VERSION_STRING})
    set(PYTHON_VERSION_MAJOR ${PYTHON3_VERSION_MAJOR})
    set(PYTHON_VERSION_MINOR ${PYTHON3_VERSION_MINOR})
    set(PYTHON_VERSION_PATCH ${PYTHON3_VERSION_PATCH})
    set(PYTHON_MODULE_PATH ${PYTHON3_MODULE_PATH})
    set(PYTHONINTERP_FOUND TRUE)
  else(${ver} EQUAL 2 AND ${PYTHON2INTERP_FOUND})
    unset(PYTHON_EXECUTABLE)
    unset(PYTHON_VERSION_STRING)
    unset(PYTHON_VERSION_MAJOR)
    unset(PYTHON_VERSION_MINOR)
    unset(PYTHON_VERSION_PATCH)
    unset(PYTHON_MODULE_PATH)
    set(PYTHONINTERP_FOUND FALSE)
  endif(${ver} EQUAL 2 AND ${PYTHON2INTERP_FOUND})
endmacro()



#macro configure files for python2 and python3
macro(configure_file_python input output)

  usePython(2)
  configure_file( ${input} py2/${output} @ONLY)

  usePython(3)
  configure_file( ${input} py3/${output} @ONLY)
  
endmacro()


#macro configure al filoes of a module for python2 and python3
macro(configure_module_python module)
  file(GLOB_RECURSE files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/${ModuleName}/ *.in)

  foreach(file ${files})
    string(REGEX REPLACE "\\.in$" "" f "${file}")

    configure_file_python(${file} ${f})

  endforeach(file ${files})
  
endmacro()


# MACRO to copy to binary directory all files of python modulefor python2 and 3 
macro(copy_to_binary_python target ModuleName)

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/py2/${ModuleName})
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/py2/${ModuleName})
  endif()
  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/py3/${ModuleName})
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/py3/${ModuleName})
  endif()


  file(GLOB_RECURSE files LIST_DIRECTORIES true RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/${ModuleName} ${ModuleName}/*)



  string(REGEX REPLACE "[^;]+\\.in;?" "" files "${files}")

  foreach(file ${files})
    if(IS_DIRECTORY ${file})
        file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/py2/${file})
        file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/py3/${file})
    else(IS_DIRECTORY ${file})
        add_custom_command(TARGET ${target} 
            COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/${ModuleName}/${file} ${CMAKE_CURRENT_BINARY_DIR}/py2/${ModuleName}/${file}
        )
        add_custom_command(TARGET ${target} 
            COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/${ModuleName}/${file} ${CMAKE_CURRENT_BINARY_DIR}/py3/${ModuleName}/${file}
        )
    endif(IS_DIRECTORY ${file})
  endforeach(file ${files})
endmacro()



#MACRO to install python modules for python2 and 3 
macro (install_python module component)
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/py2/${module}

    DESTINATION ${PYTHON2_MODULE_PATH}

    COMPONENT ${component}

  )

  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/py3/${module}

    DESTINATION ${PYTHON3_MODULE_PATH}

    COMPONENT ${component}

  )
endmacro()


# Find if a Python module is installed
# Found at http://www.cmake.org/pipermail/cmake/2011-January/041666.html
# To use do: find_python_module(PyQt5 REQUIRED)
function(find_python_module module)
  string(TOUPPER ${module} module_upper)
  if(NOT PY_${module_upper})
    if(ARGC GREATER 1 AND ARGV1 STREQUAL "REQUIRED")
      set(${module}_FIND_REQUIRED TRUE)
    endif()
    # A module's location is usually a directory, but for binary modules
    # it's a .so file.
    message(STATUS ${module})
    set(python_cmd ${PYTHON_EXECUTABLE})
    execute_process(COMMAND "${python_cmd}" "-c" 
      "import re, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__file__))"
      RESULT_VARIABLE _${module}_status  
        OUTPUT_VARIABLE _${module}_location
        ERROR_QUIET 
      OUTPUT_STRIP_TRAILING_WHITESPACE)   
    if(NOT _${module}_status )
      set(PY_${module_upper} ${_${module}_location} CACHE STRING 
        "Location of Python module ${module}")
    endif(NOT _${module}_status )
  endif(NOT PY_${module_upper})
  find_package_handle_standard_args(PY_${module} DEFAULT_MSG PY_${module_upper})
endfunction(find_python_module)
