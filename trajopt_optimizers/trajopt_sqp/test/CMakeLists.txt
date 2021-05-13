find_package(GTest REQUIRED)

if(NOT TARGET GTest::GTest)
  add_library(GTest::GTest INTERFACE IMPORTED)
  set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if (${GTEST_LIBRARIES})
    set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_LIBRARIES}")
  else()
    if (MSVC)
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "gtest.lib")
    else()
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest.so")
    endif()
  endif()
endif()

if(NOT TARGET GTest::Main)
  add_library(GTest::Main INTERFACE IMPORTED)
  set_target_properties(GTest::Main PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if (${GTEST_MAIN_LIBRARIES})
    set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_MAIN_LIBRARIES}")
  else()
    if (MSVC)
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "gtest_main.lib")
    else()
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest_main.so")
    endif()
  endif()
endif()

include(GoogleTest)

macro(add_gtest test_name test_file)
  add_executable(${test_name} ${test_file})
  target_compile_options(${test_name} PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE} ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${test_name} PRIVATE ${TRAJOPT_COMPILE_DEFINITIONS} TRAJOPT_IFOPT_DIR="${CMAKE_SOURCE_DIR}")
  target_cxx_version(${test_name} PRIVATE VERSION ${TRAJOPT_CXX_VERSION})
  target_clang_tidy(${test_name} ARGUMENTS ${TRAJOPT_CLANG_TIDY_ARGS} ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
  target_link_libraries(${test_name}
      ${PROJECT_NAME}
      tesseract::tesseract_environment_ofkt
      console_bridge
      ifopt::ifopt_ipopt
      trajopt::trajopt_ifopt
      trajopt::trajopt_test_data
      GTest::GTest
      GTest::Main
      trajopt::trajopt_test_data)
  target_include_directories(${test_name} PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
      ${GTEST_INCLUDE_DIRS})
  target_include_directories(${test_name} SYSTEM PRIVATE
      ${PCL_INCLUDE_DIRS})
  add_gtest_discover_tests(${test_name})
  add_dependencies(${test_name} ${PROJECT_NAME})
  add_dependencies(run_tests ${test_name})
endmacro()

add_gtest(${PROJECT_NAME}_joint_position_optimization_unit joint_position_optimization_unit.cpp)
add_gtest(${PROJECT_NAME}_joint_velocity_optimization_unit joint_velocity_optimization_unit.cpp)
add_gtest(${PROJECT_NAME}_cart_position_optimization_unit cart_position_optimization_unit.cpp)
add_gtest(${PROJECT_NAME}_cart_position_optimization_trajopt_sco_unit cart_position_optimization_trajopt_sco_unit.cpp)
add_gtest(${PROJECT_NAME}_cast_cost_attached_unit cast_cost_attached_unit.cpp)
add_gtest(${PROJECT_NAME}_cast_cost_octomap_unit cast_cost_octomap_unit.cpp)
add_gtest(${PROJECT_NAME}_cast_cost_unit cast_cost_unit.cpp)
add_gtest(${PROJECT_NAME}_cast_cost_world_unit cast_cost_world_unit.cpp)
add_gtest(${PROJECT_NAME}_numerical_ik_unit numerical_ik_unit.cpp)
add_gtest(${PROJECT_NAME}_planning_unit planning_unit.cpp)
add_gtest(${PROJECT_NAME}_simple_collision_unit simple_collision_unit.cpp)
