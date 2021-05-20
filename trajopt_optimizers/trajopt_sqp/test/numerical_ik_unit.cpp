/**
 * @file numerical_ik_unit.cpp
 * @brief A numerical ik unit test
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
#include <ctime>
#include <sstream>
#include <gtest/gtest.h>
#include <tesseract_common/types.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_scene_graph/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include "test_suite_utils.hpp"

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_collision;
using namespace tesseract_kinematics;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;

class NumericalIKTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter;                             /**< Trajopt Plotter */

  void SetUp() override
  {
    tesseract_common::fs::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/arm_around_table.urdf");
    tesseract_common::fs::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/pr2.srdf");

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(env->init<OFKTStateSolver>(urdf_file, srdf_file, locator));

    // Create plotting tool
    //    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));

    std::unordered_map<std::string, double> ipos;
    ipos["torso_lift_joint"] = 0.0;
    env->setState(ipos);
  }
};

TEST_F(NumericalIKTest, numerical_ik)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("PlanningTest, numerical_ik1");

  tesseract_environment::StateSolver::Ptr state_solver = env->getStateSolver();
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  auto forward_kinematics = env->getManipulatorManager()->getFwdKinematicSolver("left_arm");
  AdjacencyMap::Ptr adjacency_map = std::make_shared<AdjacencyMap>(
      env->getSceneGraph(), forward_kinematics->getActiveLinkNames(), env->getCurrentState()->link_transforms);
  Eigen::Isometry3d change_base = env->getLinkTransform(forward_kinematics->getBaseLinkName());

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0);

  // 2) Create the problem
  ifopt::Problem nlp;

  // 3) Add Variables
  Eigen::VectorXd cur_position(7);  // env->getCurrentJointValues(forward_kinematics->getJointNames());
  cur_position << 0, 0, 0, -0.001, 0, -0.001, 0;
  auto var = std::make_shared<trajopt::JointPosition>(
      cur_position, forward_kinematics->getJointNames(), forward_kinematics->getLimits(), "Joint_Position_0");
  nlp.AddVariableSet(var);

  // 4) Add constraints
  Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
  target_pose.linear() = Eigen::Quaterniond(0, 0, 1, 0).toRotationMatrix();
  target_pose.translation() = Eigen::Vector3d(0.4, 0, 0.8);

  auto kinematic_info =
      std::make_shared<CartPosKinematicInfo>(forward_kinematics, adjacency_map, change_base, "l_gripper_tool_frame");
  auto cnt = std::make_shared<trajopt::CartPosConstraint>(target_pose, kinematic_info, var);
  nlp.AddConstraintSet(cnt);

  nlp.PrintCurrent();
  std::cout << "Jacobian: \n" << nlp.GetJacobianOfConstraints() << std::endl;

  std::stringstream ss;
  ss << cur_position;
  CONSOLE_BRIDGE_logDebug("Initial Vars: %s", ss.str().c_str());

  Eigen::Isometry3d initial_pose = forward_kinematics->calcFwdKin(cur_position);
  initial_pose = change_base * initial_pose;

  ss = std::stringstream();
  ss << initial_pose.translation().transpose();
  CONSOLE_BRIDGE_logDebug("Initial Position: %s", ss.str().c_str());

  // 5) Setup solver
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(true);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
  qp_solver->solver_.settings()->setMaxIteraction(8192);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  // 6) solve
  solver.verbose = true;
  solver.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();

  EXPECT_TRUE(solver.getStatus() == trajopt_sqp::SQPStatus::NLP_CONVERGED);

  Eigen::Isometry3d final_pose = forward_kinematics->calcFwdKin(x);
  final_pose = change_base * final_pose;

  for (auto i = 0; i < 4; ++i)
  {
    for (auto j = 0; j < 4; ++j)
    {
      EXPECT_NEAR(target_pose(i, j), final_pose(i, j), 1e-3);
    }
  }

  ss = std::stringstream();
  ss << final_pose.translation().transpose();
  CONSOLE_BRIDGE_logDebug("Final Position: %s", ss.str().c_str());

  ss = std::stringstream();
  ss << x;
  CONSOLE_BRIDGE_logDebug("Final Vars: ", ss.str().c_str());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
