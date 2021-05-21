cmake_minimum_required(VERSION 3.10.0)
project(trajopt_ifopt VERSION 0.1.0 LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(trajopt_utils)
find_package(trajopt)
find_package(console_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ifopt REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(Boost REQUIRED)

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

set(TRAJOPT_IFOPT_SOURCE_FILES
    src/cartesian_position_constraint.cpp
    src/collision_types.cpp
    src/collision_utils.cpp
    src/discrete_collision_constraint.cpp
    src/discrete_collision_evaluators.cpp
    src/continuous_collision_constraint.cpp
    src/continuous_collision_evaluators.cpp
    src/inverse_kinematics_constraint.cpp
    src/joint_position_constraint.cpp
    src/joint_velocity_constraint.cpp
    src/squared_cost.cpp
    src/absolute_cost.cpp
)

add_library(${PROJECT_NAME} ${TRAJOPT_IFOPT_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC
  console_bridge::console_bridge
  ifopt::ifopt_core
  trajopt::trajopt_utils
  trajopt::trajopt
  tesseract::tesseract_environment_core
  Eigen3::Eigen
  Boost::boost)
target_compile_options(${PROJECT_NAME} PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TRAJOPT_CXX_VERSION})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

#############
## Install ##
#############

configure_package(NAMESPACE trajopt TARGETS ${PROJECT_NAME})

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
