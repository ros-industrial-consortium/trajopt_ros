cmake_minimum_required(VERSION 3.5.0)
project(Pagmo)

find_package(Pagmo QUIET)

if (NOT ${Pagmo_FOUND})
  message(WARNING "No valid Pagmo2 version found. Cloning into build directory")

  include(ExternalProject)

  ExternalProject_Add(Pagmo
    GIT_REPOSITORY    https://github.com/esa/pagmo2
    GIT_TAG           v2.15.0
    SOURCE_DIR        ${CMAKE_BINARY_DIR}/../trajopt_ext/pagmo-src
    BINARY_DIR        ${CMAKE_BINARY_DIR}/../trajopt_ext/pagmo-build
    CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DPAGMO_WITH_EIGEN3:BOOL=ON
            -DPAGMO_WITH_IPOPT:BOOL=ON
            -DPAGMO_WITH_NLOPT:BOOL=ON
  )
endif()

install(FILES package.xml DESTINATION share/Pagmo)
