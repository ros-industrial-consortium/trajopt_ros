/**
 * @file joint_term_unit.cpp
 * @brief The joint position constraint unit test
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
#include <gtest/gtest.h>
TRAJOPT_IGNORE_WARNINGS_POP
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <console_bridge/console.h>

using namespace trajopt;
using namespace std;

/** @brief Tests the Joint Position Constraint */
TEST(JointTermsUnit, joint_pos_constraint_1)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointTermsUnit, JointPosConstraint_1");

  std::vector<JointPosition::ConstPtr> position_vars;
  std::vector<std::string> joint_names(10, "name");
  Eigen::VectorXd init1(10);
  init1 << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
  position_vars.push_back(std::make_shared<JointPosition>(init1, joint_names, "test_var1"));

  Eigen::VectorXd init2(10);
  init2 << 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
  position_vars.push_back(std::make_shared<JointPosition>(init2, joint_names, "test_var2"));

  Eigen::VectorXd targets(10);
  targets << 20, 21, 22, 23, 24, 25, 26, 27, 28, 29;

  std::string name("test_cnt");
  JointPosConstraint position_cnt(targets, position_vars, name);

  EXPECT_EQ(position_cnt.GetRows(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
  EXPECT_EQ(position_cnt.GetName(), name);
  EXPECT_EQ(position_cnt.GetBounds().size(), targets.size() * static_cast<Eigen::Index>(position_vars.size()));
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
