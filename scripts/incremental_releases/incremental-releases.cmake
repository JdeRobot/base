## Incremental releases
#
# Copyright (c) 2015
# Author: Victor Arribas <v.arribas.urjc@gmail.com>
# License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>
#
# Requires:
#   * ${PROJECT_NAME}
#   * ${PROJECT_VERSION}
# Output:
#   * ${PROJECT_VERSION}

cmake_minimum_required(VERSION 2.8)

execute_process(COMMAND "${CMAKE_CURRENT_LIST_DIR}/version-upgrader.sh" ${PROJECT_NAME} ${PROJECT_VERSION}
	OUTPUT_VARIABLE out_version
	RESULT_VARIABLE status
)
message(STATUS "Incremental Release version=${out_version}")

if (status EQUAL 0)
  set(PROJECT_VERSION ${out_version})
else()
  message(WARNING "incremental_releases: something went wrong.\n ${status}")
endif()



# cleanup for include() call
unset(out_version)
unset(status)
