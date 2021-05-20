/**
 * @file utils_unit.cpp
 * @brief TrajOpt IFOPT utils unit tests
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
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <console_bridge/console.h>

using namespace trajopt;
using namespace std;

TEST(UtilsUnit, toBoundsMatrixX2d)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("UtilsUnit, toBoundsMatrixX2d");

  Eigen::MatrixX2d input_bounds(5, 2);
  input_bounds << -1, 1, -2, 2, -3, 3, -4, 4, -5, 5;
  std::vector<ifopt::Bounds> results = toBounds(input_bounds);

  // Check that the results are the correct size
  EXPECT_EQ(results.size(), input_bounds.rows());

  // Check that all bounds were set correctly
  for (Eigen::Index i = 0; i < input_bounds.rows(); i++)
  {
    EXPECT_EQ(input_bounds(i, 0), results[static_cast<std::size_t>(i)].lower_);
    EXPECT_EQ(input_bounds(i, 1), results[static_cast<std::size_t>(i)].upper_);
  }
}

TEST(UtilsUnit, toBoundsVectorXd)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("UtilsUnit, toBoundsVectorXd");

  Eigen::VectorXd lower_bounds(5);
  Eigen::VectorXd upper_bounds(5);
  lower_bounds << -1, -2, -3, -4, -5;
  upper_bounds << 1, 2, 3, 4, 5;
  std::vector<ifopt::Bounds> results = toBounds(lower_bounds, upper_bounds);

  // Check that the results are the correct size
  EXPECT_EQ(results.size(), lower_bounds.rows());

  // Check that all bounds were set correctly
  for (Eigen::Index i = 0; i < lower_bounds.rows(); i++)
  {
    EXPECT_EQ(lower_bounds(i), results[static_cast<std::size_t>(i)].lower_);
    EXPECT_EQ(upper_bounds(i), results[static_cast<std::size_t>(i)].upper_);
  }
}

TEST(UtilsUnit, interpolate)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("UtilsUnit, interpolate");

  Eigen::VectorXd start = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd end = Eigen::VectorXd::Ones(10);
  std::vector<Eigen::VectorXd> results = interpolate(start, end, 11);

  // Check size
  EXPECT_EQ(results.size(), 11);

  // Check that the start and end are correct
  EXPECT_TRUE(start.isApprox(results.front()));
  EXPECT_TRUE(end.isApprox(results.back()));

  // Check that intermediate results are correct
  EXPECT_TRUE(results[2].isApprox(Eigen::VectorXd::Ones(10) * 0.2));
  EXPECT_TRUE(results[5].isApprox(Eigen::VectorXd::Ones(10) * 0.5));
  EXPECT_TRUE(results[7].isApprox(Eigen::VectorXd::Ones(10) * 0.7));
}

/** @brief Tests getClosestValidPoint: Input within Finite bounds*/
TEST(UtilsUnit, getClosestValidPoint1)  // NOLINT
{
  ifopt::Bounds bound(-2.0, 2.0);
  std::vector<ifopt::Bounds> bounds(3, bound);
  Eigen::VectorXd input = Eigen::VectorXd::Ones(3) * 1.0;
  Eigen::VectorXd output = trajopt::getClosestValidPoint(input, bounds);
  EXPECT_TRUE(output.isApprox(input));
}
/** @brief Tests getClosestValidPoint: Input greater than Finite bounds*/
TEST(UtilsUnit, getClosestValidPoint2)  // NOLINT
{
  ifopt::Bounds bound(-2.0, 2.0);
  std::vector<ifopt::Bounds> bounds(3, bound);
  Eigen::VectorXd input = Eigen::VectorXd::Ones(3) * 3.0;
  Eigen::VectorXd output = trajopt::getClosestValidPoint(input, bounds);
  Eigen::VectorXd desired_results = Eigen::VectorXd::Ones(3) * 2.0;
  EXPECT_TRUE(output.isApprox(desired_results));
}
/** @brief Tests getClosestValidPoint: Input less than Finite bounds*/
TEST(UtilsUnit, getClosestValidPoint3)  // NOLINT
{
  ifopt::Bounds bound(-2.0, 2.0);
  std::vector<ifopt::Bounds> bounds(3, bound);
  Eigen::VectorXd input = Eigen::VectorXd::Ones(3) * -3.0;
  Eigen::VectorXd output = trajopt::getClosestValidPoint(input, bounds);
  Eigen::VectorXd desired_results = Eigen::VectorXd::Ones(3) * -2.0;
  EXPECT_TRUE(output.isApprox(desired_results));
}
/** @brief Tests getClosestValidPoint: Input within BoundGreaterZero*/
TEST(UtilsUnit, getClosestValidPoint4)  // NOLINT
{
  ifopt::Bounds bound(ifopt::BoundGreaterZero);
  std::vector<ifopt::Bounds> bounds(3, bound);
  Eigen::VectorXd input = Eigen::VectorXd::Ones(3) * 3.0e6;
  Eigen::VectorXd output = trajopt::getClosestValidPoint(input, bounds);
  EXPECT_TRUE(output.isApprox(input));
}
/** @brief Tests getClosestValidPoint: Input within BoundSmallerZero*/
TEST(UtilsUnit, getClosestValidPoint5)  // NOLINT
{
  ifopt::Bounds bound(ifopt::BoundSmallerZero);
  std::vector<ifopt::Bounds> bounds(3, bound);
  Eigen::VectorXd input = Eigen::VectorXd::Ones(3) * -3.0e6;
  Eigen::VectorXd output = trajopt::getClosestValidPoint(input, bounds);
  EXPECT_TRUE(output.isApprox(input));
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
