cmake_minimum_required(VERSION 3.10.0)
project(trajopt_sqp VERSION 0.1.0 LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(console_bridge REQUIRED)
find_package(ifopt REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_visualization REQUIRED)

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

###########
## Build ##
###########

add_library(${PROJECT_NAME}
    src/osqp_eigen_solver.cpp
    src/ifopt_qp_problem.cpp
    src/trust_region_sqp_solver.cpp
    src/trajopt_qp_problem.cpp
    src/expressions.cpp
#    src/affine_expression.cpp
#    src/quadratic_expression.cpp
#    src/expression_operations.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  console_bridge::console_bridge
  ifopt::ifopt_core
  OsqpEigen::OsqpEigen
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME} PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_DEFINITIONS})
#target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TRAJOPT_CLANG_TIDY_ARGS} ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TRAJOPT_CXX_VERSION})

add_library(${PROJECT_NAME}_callbacks
  src/callbacks/cartesian_error_plotter.cpp
  src/callbacks/clear_plotter.cpp
  src/callbacks/collision_plotter.cpp
  src/callbacks/joint_state_plotter.cpp
  src/callbacks/wait_for_input.cpp
)
target_include_directories(${PROJECT_NAME}_callbacks
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_callbacks PUBLIC
  trajopt::trajopt_ifopt
  tesseract::tesseract_visualization
  tesseract::tesseract_common
  console_bridge::console_bridge
  osqp::osqp)
target_compile_options(${PROJECT_NAME}_callbacks PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_callbacks PUBLIC ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_callbacks PUBLIC ${TRAJOPT_COMPILE_DEFINITIONS})
#target_clang_tidy(${PROJECT_NAME}_callbacks ARGUMENTS ${TRAJOPT_CLANG_TIDY_ARGS} ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_callbacks PUBLIC VERSION ${TRAJOPT_CXX_VERSION})


#############
## Install ##
#############

configure_package(NAMESPACE trajopt TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_callbacks)

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
  add_run_tests_target(ENABLE ${TRAJOPT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()
