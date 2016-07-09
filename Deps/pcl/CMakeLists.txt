# CHECK PCL

include(FindPkgConfig)
find_package(PCL QUIET)
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS})
if (PCL_INCLUDE_DIRS)
    add_definitions(${PCL_DEFINITIONS})
    set(with_pcl TRUE)

    # Only is DPKG is found
    find_program(_DPKG_CMD dpkg)
    find_program(_READLINK_CMD readlink)

    if(_DPKG_CMD AND _READLINK_CMD)

        foreach(component ${PCL_TO_FIND_COMPONENTS})
            string(TOUPPER ${component} COMPONENT)

            if(PCL_${COMPONENT}_FOUND)

                if(PCL_${COMPONENT}_LIBRARY)
                    list(GET PCL_${COMPONENT}_LIBRARY 0 _LIB)
                    execute_process(COMMAND readlink -f ${_LIB} OUTPUT_VARIABLE _LIB OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
                    execute_process(COMMAND dpkg -S ${_LIB} OUTPUT_VARIABLE _PKG)
                    string(REGEX REPLACE "^([^:]+):.*" \\1 _PKG ${_PKG})

                    list(APPEND DEPS ${_PKG})
                endif(PCL_${COMPONENT}_LIBRARY)

            endif(PCL_${COMPONENT}_FOUND)

        endforeach(component ${PCL_TO_FIND_COMPONENTS})
    else()
        message(WARNING "Package dependencies can't be determined safely in this system.")
        message(WARNING "Build will not be affected, but generated packages with CPack should be checked")

    endif(_DPKG_CMD AND _READLINK_CMD)

    list(APPEND DEPS_DEV libpcl-all)
    FIND_PATH( pcl_openni NAMES pcl/io/openni_grabber.h HINTS ${PCL_INCLUDE_DIRS})
endif()
