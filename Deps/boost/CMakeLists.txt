include(FindPkgConfig)
find_package(Boost COMPONENTS system filesystem REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

# Find related packages for this system
if(Boost_FOUND)
  # Only is DPKG is found
  find_program(_DPKG_CMD dpkg)
  find_program(_READLINK_CMD readlink)

  if(_DPKG_CMD AND _READLINK_CMD)

    foreach(component ${_Boost_COMPONENTS_SEARCHED})
        string(TOUPPER ${component} COMPONENT)

        if(Boost_${COMPONENT}_FOUND)

            if(Boost_${COMPONENT}_LIBRARY)
                list(GET Boost_${COMPONENT}_LIBRARY 0 _LIB)
                execute_process(COMMAND readlink -f ${_LIB} OUTPUT_VARIABLE _LIB OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
                execute_process(COMMAND dpkg -S ${_LIB} OUTPUT_VARIABLE _PKG)
                string(REGEX REPLACE "^([^:]+):.*" \\1 _PKG ${_PKG})
        
                list(APPEND DEPS ${_PKG})
            endif(Boost_${COMPONENT}_LIBRARY)

          endif(Boost_${COMPONENT}_FOUND)

    endforeach(component ${Boost_FIND_COMPONENTS})
  else()
      message(WARNING "Package dependencies can't be determined safely in this system.")
      message(WARNING "Build will not be affected, but generated packages with CPack should be checked")

  endif(_DPKG_CMD AND _READLINK_CMD)

endif(Boost_FOUND)

# ubuntu

list(APPEND DEPS libboost-system1.54.0 libboost-filesystem1.54.0)
