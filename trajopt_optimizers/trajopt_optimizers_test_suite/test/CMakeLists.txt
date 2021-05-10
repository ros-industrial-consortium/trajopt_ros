find_package(GTest REQUIRED)

macro(add_gtest test_name test_file)
  add_executable(${test_name} ${test_file})
  trajopt_target_compile_options(${test_name} PRIVATE)
  trajopt_clang_tidy(${test_name})
  target_compile_definitions(${test_name} PRIVATE TRAJOPT_IFOPT_DIR="${CMAKE_SOURCE_DIR}")
  target_link_libraries(${test_name}
      ${PROJECT_NAME}
      ifopt::ifopt_ipopt
      trajopt::trajopt_ifopt
      trajopt::trajopt_test_data
      trajopt::trajopt_sqp
      trajopt::trajopt_pagmo
      Pagmo::pagmo
      GTest::GTest
      GTest::Main)
  target_include_directories(${test_name} PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
      ${GTEST_INCLUDE_DIRS})
  target_include_directories(${test_name} SYSTEM PRIVATE
      ${PCL_INCLUDE_DIRS})
  trajopt_gtest_discover_tests(${test_name})
  add_dependencies(${test_name} ${PROJECT_NAME})
  add_dependencies(run_tests ${test_name})
endmacro()

add_gtest(${PROJECT_NAME}_joint_position_optimization_unit joint_position_optimization_unit.cpp)
add_gtest(${PROJECT_NAME}_cart_position_optimization_unit cart_position_optimization_unit.cpp)
add_gtest(${PROJECT_NAME}_basic_optimization_unit basic_optimization_unit.cpp)
add_gtest(${PROJECT_NAME}_single_optimization_unit single_optimization_unit.cpp)
