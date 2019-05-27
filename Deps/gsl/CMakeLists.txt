PKG_CHECK_MODULES(gsl REQUIRED gsl)
include_directories(${gsl_INCLUDE_DIRS})
link_directories(${gsl_LIBRARIES_DIRS})

list(APPEND DEPS libgsl23 gsl-bin)
list(APPEND DEPS_DEV libgsl-dev)