/**
 * @file variable_sets_unit.cpp
 * @brief TrajOpt IFOPT variable sets unit tests
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
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <console_bridge/console.h>

using namespace trajopt_ifopt;
using namespace std;

/**
 * @brief Tests the joint position variable
 */
TEST(VariableSetsUnit, joint_position_1)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("VariableSetsUnit, joint_position_1");

  std::string name("test_var");
  Eigen::VectorXd init(10);
  init << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
  JointPosition position_var(init, std::vector<std::string>(), name);

  // Check that everything has been initialized correctly
  EXPECT_EQ(position_var.GetRows(), init.size());
  EXPECT_EQ(position_var.GetName(), name);
  EXPECT_EQ(position_var.GetBounds().size(), init.size());
  EXPECT_TRUE(init.isApprox(position_var.GetValues()));

  // Check that setting variables works
  Eigen::VectorXd changed(10);
  changed << 10, 11, 12, 13, 14, 15, 16, 17, 18, 19;
  position_var.SetVariables(changed);
  EXPECT_TRUE(changed.isApprox(position_var.GetValues()));

  // Check that setting bounds works
  ifopt::Bounds bounds(-0.1234, 0.5678);
  std::vector<ifopt::Bounds> bounds_vec = std::vector<ifopt::Bounds>(static_cast<size_t>(init.size()), bounds);
  position_var.SetBounds(bounds_vec);
  std::vector<ifopt::Bounds> results_vec = position_var.GetBounds();
  for (size_t i = 0; i < bounds_vec.size(); i++)
  {
    EXPECT_EQ(bounds_vec[i].lower_, results_vec[i].lower_);
    EXPECT_EQ(bounds_vec[i].upper_, results_vec[i].upper_);
  }
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
