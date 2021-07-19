/**
 * @file cart_position_optimization_trajopt_sco_unit.cpp
 * @brief This is example is made to pair with cart_position_example.cpp.
 * This is the same motion planning problem in the trajopt_sco framework
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_common/types.h>
#include <tesseract_common/macros.h>
#include <json/json.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/utils.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;

const bool DEBUG = true;

inline std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://trajopt") == 0)
  {
    mod_url.erase(0, strlen("package://trajopt"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TRAJOPT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

// This is example is made to pair with cart_position_example.cpp. This is the same motion planning problem in the
// trajopt_sco framework
TEST(CartPositionOptimizationTrajoptSCO, cart_position_optimization_trajopt_sco)  // NOLINT
{
  if (DEBUG)
  {
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    util::gLogLevel = util::LevelInfo;
  }
  else
  {
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);
    util::gLogLevel = util::LevelError;
  }

  // 1)  Load Robot
  tesseract_common::fs::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/arm_around_table.urdf");
  tesseract_common::fs::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/pr2.srdf");
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  auto env = std::make_shared<Environment>();
  env->init<OFKTStateSolver>(urdf_file, srdf_file, locator);

  // Extract necessary kinematic information
  auto forward_kinematics = env->getManipulatorManager()->getFwdKinematicSolver("right_arm");
  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), forward_kinematics->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  ProblemConstructionInfo pci(env);

  // Populate Basic Info
  pci.basic_info.n_steps = 1;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = pci.getManipulator(pci.basic_info.manip);

  // Populate Init Info
  EnvState::ConstPtr current_state = pci.env->getCurrentState();
  Eigen::VectorXd start_pos(forward_kinematics->numJoints());
  start_pos << 0, 0, 0, -1.0, 0, -1, 0.0;
  if (DEBUG)
    std::cout << "Joint Limits:\n" << forward_kinematics->getLimits().joint_limits.transpose() << std::endl;

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = tesseract_common::TrajArray(1, pci.kin->numJoints());
  auto zero = Eigen::VectorXd::Zero(7);
  pci.init_info.data = zero.transpose();

  auto joint_target = start_pos;
  auto target_pose = forward_kinematics->calcFwdKin(joint_target);
  auto world_to_base = pci.env->getCurrentState()->link_transforms.at(forward_kinematics->getBaseLinkName());
  target_pose = world_to_base * target_pose;

  if (DEBUG)
    std::cout << "target_pose:\n" << target_pose.matrix() << std::endl;

  {
    auto pose = std::make_shared<CartPoseTermInfo>();
    pose->term_type = TT_CNT;
    pose->name = "waypoint_cart_0";
    pose->link = "r_gripper_tool_frame";
    pose->timestep = 0;

    pose->xyz = target_pose.translation();
    Eigen::Quaterniond q(target_pose.linear());
    pose->wxyz = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
    pose->pos_coeffs = Eigen::Vector3d(1, 1, 1);
    pose->rot_coeffs = Eigen::Vector3d(1, 1, 1);

    pci.cnt_infos.push_back(pose);
  }

  pci.basic_info.convex_solver = sco::ModelType::OSQP;

  auto prob = ConstructProblem(pci);

  sco::BasicTrustRegionSQP opt(prob);

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  tesseract_common::TrajArray traj = getTraj(opt.x(), prob->GetVars());

  for (Eigen::Index i = 0; i < traj.cols(); i++)
    EXPECT_NEAR(traj(0, i), joint_target[i], 1.1e-3);

  if (DEBUG)
  {
    std::cout << "Initial: " << prob->GetInitTraj() << std::endl;
    std::cout << "Results: " << traj << std::endl;
  }
}
