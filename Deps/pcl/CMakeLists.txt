# CHECK PCL

include(FindPkgConfig)
find_package(PCL QUIET)
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS})
if (PCL_INCLUDE_DIRS)
    add_definitions(${PCL_DEFINITIONS})
    set(with_pcl TRUE)
    SET(DEPS "${DEPS} libboost-system1.46.1, libboost-filesystem1.46.1,")
    SET(DEPS "${DEPS} libpcl-visualization-1.7, libpcl-surface-1.7, libpcl-registration-1.7, libpcl-segmentation-1.7, libpcl-filters-1.7, libpcl-sample-consensus-1.7, libpcl-io-1.7, libpcl-octree-1.7, libpcl-features-1.7, libpcl-kdtree-1.7, libpcl-common-1.7,")
    FIND_PATH( pcl_openni NAMES pcl/io/openni_grabber.h HINTS ${PCL_INCLUDE_DIRS})
endif()
