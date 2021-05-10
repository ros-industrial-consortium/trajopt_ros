cmake_minimum_required(VERSION 3.5.0)
project(trajopt_optimizers_test_suite VERSION 0.1.0 LANGUAGES CXX)

find_package(console_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ifopt REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Pagmo REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(trajopt_pagmo REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE
    console_bridge
    trajopt::trajopt_utils
)
trajopt_target_compile_options(${PROJECT_NAME} INTERFACE)
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

 #############
 ## Testing ##
 #############

 if (TRAJOPT_ENABLE_TESTING)
   enable_testing()
   add_custom_target(run_tests ALL
       WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
       COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -V)
   add_subdirectory(test)
 endif()
