cmake_minimum_required(VERSION 3.5.0)
project(NLopt)

find_package(NLopt QUIET)

if (NOT ${NLopt_FOUND})
  message(WARNING "No valid NLopt version found. Cloning into build directory")

  include(ExternalProject)

  ExternalProject_Add(NLopt
    GIT_REPOSITORY    https://github.com/stevengj/nlopt.git
    GIT_TAG           v2.6.2
    SOURCE_DIR        ${CMAKE_BINARY_DIR}/../trajopt_ext/nlopt-src
    BINARY_DIR        ${CMAKE_BINARY_DIR}/../trajopt_ext/nlopt-build
    CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE:STRING=Release
  )
endif()

install(FILES package.xml DESTINATION share/NLopt)
