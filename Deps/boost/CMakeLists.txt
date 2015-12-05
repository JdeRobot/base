include(FindPkgConfig)
find_package(Boost COMPONENTS system filesystem REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

# ubuntu

list(APPEND DEPS libboost-system1.54.0 libboost-filesystem1.54.0)
