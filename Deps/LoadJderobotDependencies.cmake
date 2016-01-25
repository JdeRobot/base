#  Copyright (C) 2016 JdeRobot Developers Team
#  Authors:
#    Victor Arribas <v.arribas.urjc@gmail.com>


## Alphabetical sorted to handle dependency order
# this means that you must to prepend letter based sort at directory level
## To disable a dependency rename CMaleLists.txt (for example: add .disabled)
file(GLOB_RECURSE CMakeLists_files "${CMAKE_CURRENT_LIST_DIR}/**/CMakeLists.txt")
list(SORT CMakeLists_files)
foreach(CMakeLists_file ${CMakeLists_files})
	include(${CMakeLists_file})
endforeach()
