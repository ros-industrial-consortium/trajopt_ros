/**
 * @file cart_position_optimization_unit.cpp
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

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_common/types.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>

const bool DEBUG = false;

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

class CartPositionOptimization : public testing::TestWithParam<const char*>
{
public:
  // 1) Create the problem
  ifopt::Problem nlp_;

  Eigen::VectorXd joint_target_;

  void SetUp() override
  {
    if (DEBUG)
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    else
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);

    // 1)  Load Robot
    tesseract_common::fs::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/arm_around_table.urdf");
    tesseract_common::fs::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/pr2.srdf");
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    auto env = std::make_shared<tesseract_environment::Environment>();
    env->init<tesseract_environment::OFKTStateSolver>(urdf_file, srdf_file, locator);

    // Extract necessary kinematic information
    auto forward_kinematics = env->getManipulatorManager()->getFwdKinematicSolver("right_arm");
    auto world_to_base = env->getCurrentState()->link_transforms.at(forward_kinematics->getBaseLinkName());
    tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
        env->getSceneGraph(), forward_kinematics->getActiveLinkNames(), env->getCurrentState()->link_transforms);
    auto kinematic_info =
        std::make_shared<trajopt_ifopt::KinematicsInfo>(forward_kinematics, adjacency_map, world_to_base);

    // Get target position
    Eigen::VectorXd start_pos(forward_kinematics->numJoints());
    start_pos << 0.0, 0, 0, -1.0, 0, -1, -0.00;
    if (DEBUG)
      std::cout << "Joint Limits:\n" << forward_kinematics->getLimits().joint_limits.transpose() << std::endl;

    joint_target_ = start_pos;
    Eigen::Isometry3d target_pose = forward_kinematics->calcFwdKin(joint_target_);
    target_pose = world_to_base * target_pose;

    // 3) Add Variables
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
    for (int ind = 0; ind < 1; ind++)
    {
      auto zero = Eigen::VectorXd::Zero(7);
      auto var = std::make_shared<trajopt_ifopt::JointPosition>(
          zero, forward_kinematics->getJointNames(), "Joint_Position_" + std::to_string(ind));
      vars.push_back(var);
      nlp_.AddVariableSet(var);
    }

    // 4) Add constraints
    for (const auto& var : vars)
    {
      trajopt_ifopt::CartPosInfo cart_info(kinematic_info, target_pose, forward_kinematics->getTipLinkName());
      auto cnt = std::make_shared<trajopt_ifopt::CartPosConstraint>(cart_info, var);
      nlp_.AddConstraintSet(cnt);
    }

    if (DEBUG)
    {
      nlp_.PrintCurrent();
      std::cout << "Constraint Jacobian: \n" << nlp_.GetJacobianOfConstraints() << std::endl;
    }
  }
};

/**
 * @brief Applies a cartesian position constraint and solves the problem with trajopt_sqp
 */
TEST_F(CartPositionOptimization, cart_position_optimization_trajopt_sqp)  // NOLINT
{
  ifopt::Problem nlp_trajopt_sqp(nlp_);
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(DEBUG);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
  qp_solver->solver_.settings()->setMaxIteration(8192);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  // solve
  solver.verbose = DEBUG;
  solver.Solve(nlp_trajopt_sqp);
  Eigen::VectorXd x = nlp_trajopt_sqp.GetOptVariables()->GetValues();

  for (Eigen::Index i = 0; i < joint_target_.size(); i++)
    EXPECT_NEAR(x[i], joint_target_[i], 1.1e-3);

  if (DEBUG)
  {
    std::cout << x.transpose() << std::endl;
    nlp_trajopt_sqp.PrintCurrent();
  }
}
