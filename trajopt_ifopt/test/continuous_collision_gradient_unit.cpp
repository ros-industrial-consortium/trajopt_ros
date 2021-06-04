/**
 * @file continuous_collision_gradient_unit.cpp
 * @brief  Unit test for the different methods for combining the gradients
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date June 1, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <gtest/gtest.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_scene_graph/utils.h>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_test_utils.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>
#include <trajopt_ifopt/constraints/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;

class ContinuousCollisionGradientTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter_;                            /**< Plotter */

  void SetUp() override
  {
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/spherebot.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/spherebot.srdf");

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(env->init<OFKTStateSolver>(urdf_file, srdf_file, locator));

    gLogLevel = util::LevelError;
  }
};

void runContinuousGradientTest(const Environment::Ptr& env, double coeff, CombineCollisionDataMethod method)
{
  std::unordered_map<std::string, double> ipos;
  ipos["spherebot_x_joint"] = -1.9;
  ipos["spherebot_y_joint"] = 0;
  env->setState(ipos);

  //  plotter_->plotScene();

  std::vector<ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = env->getStateSolver();
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  auto forward_kinematics = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  AdjacencyMap::Ptr adjacency_map = std::make_shared<AdjacencyMap>(
      env->getSceneGraph(), forward_kinematics->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0);

  collisions.clear();

  // 2) Create the problem
  ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<trajopt::JointPosition::ConstPtr> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    positions.push_back(pos);
    auto var = std::make_shared<trajopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_0");
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 0, 1.9;
    positions.push_back(pos);
    auto var = std::make_shared<trajopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_1");
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 1.9, 3.8;
    positions.push_back(pos);
    auto var = std::make_shared<trajopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_2");
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }

  // Step 3: Setup collision
  auto kin = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  double margin_coeff = coeff;
  double margin = 0.02;
  auto trajopt_collision_config = std::make_shared<trajopt::TrajOptCollisionConfig>(margin, margin_coeff);
  trajopt_collision_config->collision_margin_buffer = 0.05;

  auto collision_cache = std::make_shared<trajopt::CollisionCache>(100);
  for (std::size_t i = 1; i < (vars.size() - 1); ++i)
  {
    auto collision_evaluator = std::make_shared<trajopt::LVSContinuousCollisionEvaluator>(
        collision_cache, kin, env, adj_map, Eigen::Isometry3d::Identity(), trajopt_collision_config);

    std::array<JointPosition::ConstPtr, 3> position_vars{ vars[i - 1], vars[i], vars[i + 1] };
    auto cnt = std::make_shared<trajopt::ContinuousCollisionConstraintIfopt>(
        collision_evaluator, trajopt::ContinuousCombineCollisionData(method), position_vars);
    nlp.AddConstraintSet(cnt);
  }

  std::cout << "Jacobian: \n" << nlp.GetJacobianOfConstraints().toDense() << std::endl;
  std::vector<double> init_vals{ -1.9, 0, 0, 1.9, 1.9, 3.8 };
  trajopt::Jacobian num_jac_block = calcNumericalConstraintGradient(init_vals.data(), nlp, 1e-8);
  std::cout << "Numerical Jacobian: \n" << num_jac_block.toDense() << std::endl;
}

TEST_F(ContinuousCollisionGradientTest, SUM)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("ContinuousCollisionGradientTest, SUM");
  runContinuousGradientTest(env, 1, CombineCollisionDataMethod::SUM);
  runContinuousGradientTest(env, 10, CombineCollisionDataMethod::SUM);
}
TEST_F(ContinuousCollisionGradientTest, WEIGHTED_SUM)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("ContinuousCollisionGradientTest, WEIGHTED_SUM");
  runContinuousGradientTest(env, 1, CombineCollisionDataMethod::WEIGHTED_SUM);
  runContinuousGradientTest(env, 10, CombineCollisionDataMethod::WEIGHTED_SUM);
}
TEST_F(ContinuousCollisionGradientTest, AVERAGE)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("ContinuousCollisionGradientTest, AVERAGE");
  runContinuousGradientTest(env, 1, CombineCollisionDataMethod::AVERAGE);
  runContinuousGradientTest(env, 10, CombineCollisionDataMethod::AVERAGE);
}
TEST_F(ContinuousCollisionGradientTest, WEIGHTED_AVERAGE)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("ContinuousCollisionGradientTest, WEIGHTED_AVERAGE");
  runContinuousGradientTest(env, 1, CombineCollisionDataMethod::WEIGHTED_AVERAGE);
  runContinuousGradientTest(env, 10, CombineCollisionDataMethod::WEIGHTED_AVERAGE);
}
TEST_F(ContinuousCollisionGradientTest, LEAST_SQUARES)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("ContinuousCollisionGradientTest, LEAST_SQUARES");
  runContinuousGradientTest(env, 1, CombineCollisionDataMethod::LEAST_SQUARES);
  runContinuousGradientTest(env, 10, CombineCollisionDataMethod::LEAST_SQUARES);
}
TEST_F(ContinuousCollisionGradientTest, WEIGHTED_LEAST_SQUARES)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("ContinuousCollisionGradientTest, WEIGHTED_LEAST_SQUARES");
  runContinuousGradientTest(env, 1, CombineCollisionDataMethod::WEIGHTED_LEAST_SQUARES);
  runContinuousGradientTest(env, 10, CombineCollisionDataMethod::WEIGHTED_LEAST_SQUARES);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
