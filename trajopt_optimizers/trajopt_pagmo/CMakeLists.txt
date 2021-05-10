cmake_minimum_required(VERSION 3.5.0)
project(trajopt_pagmo VERSION 0.1.0 LANGUAGES CXX)

find_package(console_bridge REQUIRED)
find_package(Pagmo REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(ifopt REQUIRED)
find_package(Eigen3 REQUIRED)


###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
    src/pagmo_problem_interface.cpp
    src/pagmo_solver.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    ifopt::ifopt_core
    console_bridge
    Pagmo::pagmo
)
trajopt_target_compile_options(${PROJECT_NAME} PUBLIC)
trajopt_clang_tidy(${PROJECT_NAME})
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})


#############
## Install ##
#############

trajopt_configure_package(${PACKAGE_LIBRARIES})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )
