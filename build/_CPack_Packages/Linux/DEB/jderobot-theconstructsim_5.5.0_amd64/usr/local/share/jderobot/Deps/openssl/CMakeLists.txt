# Search OpenSSL
find_package(PkgConfig REQUIRED)
pkg_search_module(OPENSSL REQUIRED openssl)

if( OPENSSL_FOUND )
    include_directories(${OPENSSL_INCLUDE_DIRS})
    message(STATUS "Using OpenSSL ${OPENSSL_VERSION}")
endif()

INCLUDE_DIRECTORIES(${OPENSSL_INCLUDE})

#target_link_libraries(${YOUR_TARGET_HERE} ${OPENSSL_LIBRARIES})
