PKG_CHECK_MODULES(eigen REQUIRED eigen3)
include_directories(${eigen_INCLUDE_DIRS})

list(APPEND DEPS_DEV libeigen3-dev )