cmake_minimum_required(VERSION 3.10.0)
project(trajopt VERSION 0.1.0 LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(console_bridge REQUIRED)
find_package(trajopt_sco REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if (NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

# Load variable for clang tidy args, compiler options and cxx version
trajopt_variables()

link_directories(
    /usr/local/lib
    /usr/local/lib64
)

set(TRAJOPT_SOURCE_FILES
    src/trajectory_costs.cpp
    src/kinematic_terms.cpp
    src/collision_terms.cpp
    src/json_marshal.cpp
    src/problem_description.cpp
    src/utils.cpp
    src/plot_callback.cpp
    src/file_write_callback.cpp
)

add_library(${PROJECT_NAME} ${TRAJOPT_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC
    trajopt::trajopt_sco
    trajopt::trajopt_utils
    tesseract::tesseract_environment_core
    tesseract::tesseract_visualization
    console_bridge::console_bridge
    Boost::boost
    Eigen3::Eigen
    jsoncpp_lib)
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TRAJOPT_CXX_VERSION})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

# Install test data directory for use in other packages
add_library(${PROJECT_NAME}_test_data INTERFACE)
target_link_libraries(${PROJECT_NAME}_test_data INTERFACE)
target_compile_definitions(${PROJECT_NAME}_test_data INTERFACE ${TRAJOPT_COMPILE_DEFINITIONS} TRAJOPT_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
target_cxx_version(${PROJECT_NAME}_test_data INTERFACE VERSION ${TRAJOPT_CXX_VERSION})

configure_package(NAMESPACE trajopt TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_test_data)

foreach(dir test/data)
   install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hxx"
  PATTERN ".svn" EXCLUDE
 )

if (TRAJOPT_ENABLE_TESTING)
  enable_testing()
#  add_run_tests_target(ENABLE ${TRAJOPT_ENABLE_RUN_TESTING})
  add_custom_target(run_tests)
  add_subdirectory(test)
endif()

if (TRAJOPT_ENABLE_BENCHMARKING)
  add_subdirectory(test/benchmarks)
endif()

