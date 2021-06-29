/**
 * @file planning_unit.cpp
 * @brief A PR2 arm planning around a table unit test
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
#include <memory>
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
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include "test_suite_utils.hpp"

using namespace trajopt_ifopt;
using namespace tesseract_environment;
using namespace tesseract_collision;
using namespace tesseract_kinematics;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;

class PlanningTest : public testing::TestWithParam<const char*>
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

TEST_F(PlanningTest, arm_around_table)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("PlanningTest, arm_around_table");

  std::unordered_map<std::string, double> ipos;
  ipos["torso_lift_joint"] = 0;
  ipos["r_shoulder_pan_joint"] = -1.832;
  ipos["r_shoulder_lift_joint"] = -0.332;
  ipos["r_upper_arm_roll_joint"] = -1.011;
  ipos["r_elbow_flex_joint"] = -1.437;
  ipos["r_forearm_roll_joint"] = -1.1;
  ipos["r_wrist_flex_joint"] = -1.926;
  ipos["r_wrist_roll_joint"] = 3.074;
  env->setState(ipos);

  std::vector<ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = env->getStateSolver();
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  auto forward_kinematics = env->getManipulatorManager()->getFwdKinematicSolver("right_arm");
  AdjacencyMap::Ptr adjacency_map = std::make_shared<AdjacencyMap>(
      env->getSceneGraph(), forward_kinematics->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0);

  // Initial trajectory
  tesseract_common::TrajArray trajectory(6, 7);
  trajectory.row(0) << -1.832, -0.332, -1.011, -1.437, -1.1, -1.926, 3.074;
  trajectory.row(1) << -1.411, 0.028, -0.764, -1.463, -1.525, -1.698, 3.055;
  trajectory.row(2) << -0.99, 0.388, -0.517, -1.489, -1.949, -1.289, 3.036;
  trajectory.row(3) << -0.569, 0.747, -0.27, -1.515, -2.374, -0.881, 3.017;
  trajectory.row(4) << -0.148, 1.107, -0.023, -1.541, -2.799, -0.472, 2.998;
  trajectory.row(5) << 0.062, 1.287, 0.1, -1.554, -3.011, -0.268, 2.988;

  // Create the problem
  ifopt::Problem nlp;

  // Add Variables
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    auto var = std::make_shared<trajopt_ifopt::JointPosition>(
        trajectory.row(i), forward_kinematics->getJointNames(), "Joint_Position_" + std::to_string(i));
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }

  double margin_coeff = 50;
  double margin = 0.025;
  auto trajopt_collision_config = std::make_shared<trajopt_ifopt::TrajOptCollisionConfig>(margin, margin_coeff);
  trajopt_collision_config->collision_margin_buffer = 0.02;
  trajopt_collision_config->longest_valid_segment_length = 0.005;

  // Add costs
  {
    auto cnt = std::make_shared<JointVelConstraint>(Eigen::VectorXd::Zero(7), vars);
    cnt->LinkWithVariables(nlp.GetOptVariables());
    auto cost = std::make_shared<SquaredCost>(cnt);
    nlp.AddCostSet(cost);
  }

  // Add constraints
  {  // Fix start position
    std::vector<JointPosition::ConstPtr> fixed_vars = { vars[0] };
    auto cnt = std::make_shared<JointPosConstraint>(trajectory.row(0), fixed_vars);
    nlp.AddConstraintSet(cnt);
  }

  {  // Fix end position
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> fixed_vars = { vars[5] };
    auto cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(trajectory.row(5), fixed_vars);
    nlp.AddConstraintSet(cnt);
  }

  auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
  for (std::size_t i = 1; i < (vars.size() - 1); ++i)
  {
    auto collision_evaluator =
        std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(collision_cache,
                                                                         forward_kinematics,
                                                                         env,
                                                                         adjacency_map,
                                                                         Eigen::Isometry3d::Identity(),
                                                                         trajopt_collision_config);

    std::array<JointPosition::ConstPtr, 3> position_vars{ vars[i - 1], vars[i], vars[i + 1] };
    auto cnt = std::make_shared<trajopt_ifopt::ContinuousCollisionConstraintIfopt>(
        collision_evaluator,
        trajopt_ifopt::ContinuousCombineCollisionData(CombineCollisionDataMethod::WEIGHTED_AVERAGE),
        position_vars);
    nlp.AddConstraintSet(cnt);
  }

  nlp.PrintCurrent();
  std::cout << "Jacobian: \n" << nlp.GetJacobianOfConstraints() << std::endl;

  // Setup solver
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(true);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
  qp_solver->solver_.settings()->setMaxIteration(8192);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  // 6) solve
  solver.verbose = true;
  solver.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << x.transpose() << std::endl;

  EXPECT_TRUE(solver.getStatus() == trajopt_sqp::SQPStatus::NLP_CONVERGED);

  Eigen::Map<tesseract_common::TrajArray> results(x.data(), 6, 7);

  tesseract_collision::CollisionCheckConfig config;
  config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  bool found =
      checkTrajectory(collisions, *manager, *state_solver, forward_kinematics->getJointNames(), trajectory, config);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  collisions.clear();
  found = checkTrajectory(collisions, *manager, *state_solver, forward_kinematics->getJointNames(), results, config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
