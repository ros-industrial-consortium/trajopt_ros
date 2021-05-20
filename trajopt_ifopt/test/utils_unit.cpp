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
#include <trajopt/utils.hpp>
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

/** @brief Tests calcRotationalError which return angle between [-PI, PI]*/
TEST(UtilsUnit, calcRotationalError)  // NOLINT
{
  Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pi_rot = identity * Eigen::AngleAxisd(M_PI - 0.0001, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d rot_err = trajopt::calcRotationalError(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI - 0.0001, 1e-6);
  EXPECT_TRUE(rot_err.normalized().isApprox(Eigen::Vector3d::UnitZ(), 1e-6));

  pi_rot = identity * Eigen::AngleAxisd(-M_PI + 0.0001, Eigen::Vector3d::UnitZ());
  rot_err = trajopt::calcRotationalError(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI - 0.0001, 1e-6);
  EXPECT_TRUE(rot_err.normalized().isApprox(-Eigen::Vector3d::UnitZ(), 1e-6));

  // Test greater than PI
  pi_rot = identity * Eigen::AngleAxisd(3 * M_PI_2, Eigen::Vector3d::UnitZ());
  rot_err = trajopt::calcRotationalError(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI_2, 1e-6);
  EXPECT_TRUE(rot_err.normalized().isApprox(-Eigen::Vector3d::UnitZ(), 1e-6));

  // Test lessthan than -PI
  pi_rot = identity * Eigen::AngleAxisd(-3 * M_PI_2, Eigen::Vector3d::UnitZ());
  rot_err = trajopt::calcRotationalError(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI_2, 1e-6);
  EXPECT_TRUE(rot_err.normalized().isApprox(Eigen::Vector3d::UnitZ(), 1e-6));

  // Test for angle between [0, PI]
  Eigen::Isometry3d pi_rot_plus = identity * Eigen::AngleAxisd(M_PI_2 + 0.001, Eigen::Vector3d::UnitZ());
  Eigen::Isometry3d pi_rot_minus = identity * Eigen::AngleAxisd(M_PI_2 - 0.001, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d pi_rot_delta =
      trajopt::calcRotationalError(pi_rot_plus.rotation()) - trajopt::calcRotationalError(pi_rot_minus.rotation());
  EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);

  // Test for angle between [-PI, 0]
  pi_rot_plus = identity * Eigen::AngleAxisd(-M_PI_2 + 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_minus = identity * Eigen::AngleAxisd(-M_PI_2 - 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_delta =
      trajopt::calcRotationalError(pi_rot_plus.rotation()) - trajopt::calcRotationalError(pi_rot_minus.rotation());
  EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);

  // Test for angle at 0
  pi_rot_plus = identity * Eigen::AngleAxisd(0.001, Eigen::Vector3d::UnitZ());
  pi_rot_minus = identity * Eigen::AngleAxisd(-0.001, Eigen::Vector3d::UnitZ());
  pi_rot_delta =
      trajopt::calcRotationalError(pi_rot_plus.rotation()) - trajopt::calcRotationalError(pi_rot_minus.rotation());
  EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);

  // Test for angle at PI
  pi_rot_plus = identity * Eigen::AngleAxisd(M_PI + 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_minus = identity * Eigen::AngleAxisd(M_PI - 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_delta =
      trajopt::calcRotationalError(pi_rot_plus.rotation()) - trajopt::calcRotationalError(pi_rot_minus.rotation());
  EXPECT_TRUE(pi_rot_delta.norm() > M_PI);  // This is because calcRotationalError breaks down at PI or -PI

  // Test for angle at -PI
  pi_rot_plus = identity * Eigen::AngleAxisd(-M_PI + 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_minus = identity * Eigen::AngleAxisd(-M_PI - 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_delta =
      trajopt::calcRotationalError(pi_rot_plus.rotation()) - trajopt::calcRotationalError(pi_rot_minus.rotation());
  EXPECT_TRUE(pi_rot_delta.norm() > M_PI);  // This is because calcRotationalError breaks down at PI or -PI

  // Test random axis
  for (int i = 0; i < 100; i++)
  {
    Eigen::Vector3d axis = Eigen::Vector3d::Random().normalized();

    // Avoid M_PI angle because this breaks down
    Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(100, -M_PI + 0.005, M_PI - 0.005);
    for (Eigen::Index j = 0; j < angles.rows(); j++)
    {
      pi_rot_plus = identity * Eigen::AngleAxisd(angles(j) + 0.001, axis);
      pi_rot_minus = identity * Eigen::AngleAxisd(angles(j) - 0.001, axis);
      Eigen::Vector3d e1 = trajopt::calcRotationalError(pi_rot_plus.rotation());
      Eigen::Vector3d e2 = trajopt::calcRotationalError(pi_rot_minus.rotation());
      EXPECT_FALSE((e1.norm() < -M_PI));
      EXPECT_FALSE((e1.norm() > M_PI));
      EXPECT_FALSE((e2.norm() < -M_PI));
      EXPECT_FALSE((e2.norm() > M_PI));
      pi_rot_delta = e1 - e2;
      EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);
    }

    // Avoid M_PI angle because this breaks down
    angles = Eigen::VectorXd::LinSpaced(100, M_PI + 0.005, 2 * M_PI);
    for (Eigen::Index j = 0; j < angles.rows(); j++)
    {
      pi_rot_plus = identity * Eigen::AngleAxisd(angles(j) + 0.001, axis);
      pi_rot_minus = identity * Eigen::AngleAxisd(angles(j) - 0.001, axis);
      Eigen::Vector3d e1 = trajopt::calcRotationalError(pi_rot_plus.rotation());
      Eigen::Vector3d e2 = trajopt::calcRotationalError(pi_rot_minus.rotation());
      EXPECT_FALSE((e1.norm() < -M_PI));
      EXPECT_FALSE((e1.norm() > M_PI));
      EXPECT_FALSE((e2.norm() < -M_PI));
      EXPECT_FALSE((e2.norm() > M_PI));
      pi_rot_delta = e1 - e2;
      EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);
    }

    // Avoid M_PI angle because this breaks down
    angles = Eigen::VectorXd::LinSpaced(100, -M_PI - 0.005, -2 * M_PI);
    for (Eigen::Index j = 0; j < angles.rows(); j++)
    {
      pi_rot_plus = identity * Eigen::AngleAxisd(angles(j) + 0.001, axis);
      pi_rot_minus = identity * Eigen::AngleAxisd(angles(j) - 0.001, axis);
      Eigen::Vector3d e1 = trajopt::calcRotationalError(pi_rot_plus.rotation());
      Eigen::Vector3d e2 = trajopt::calcRotationalError(pi_rot_minus.rotation());
      EXPECT_FALSE((e1.norm() < -M_PI));
      EXPECT_FALSE((e1.norm() > M_PI));
      EXPECT_FALSE((e2.norm() < -M_PI));
      EXPECT_FALSE((e2.norm() > M_PI));
      pi_rot_delta = e1 - e2;
      EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);
    }

    // These should fail
    {
      pi_rot_plus = identity * Eigen::AngleAxisd(M_PI + 0.001, axis);
      pi_rot_minus = identity * Eigen::AngleAxisd(M_PI - 0.001, axis);
      Eigen::Vector3d e1 = trajopt::calcRotationalError(pi_rot_plus.rotation());
      Eigen::Vector3d e2 = trajopt::calcRotationalError(pi_rot_minus.rotation());
      EXPECT_FALSE((e1.norm() < -M_PI));
      EXPECT_FALSE((e1.norm() > M_PI));
      EXPECT_FALSE((e2.norm() < -M_PI));
      EXPECT_FALSE((e2.norm() > M_PI));
      pi_rot_delta = e1 - e2;
      EXPECT_TRUE(pi_rot_delta.norm() > M_PI);  // This is because calcRotationalError breaks down at PI or -PI
    }
    {
      pi_rot_plus = identity * Eigen::AngleAxisd(-M_PI + 0.001, axis);
      pi_rot_minus = identity * Eigen::AngleAxisd(-M_PI - 0.001, axis);
      Eigen::Vector3d e1 = trajopt::calcRotationalError(pi_rot_plus.rotation());
      Eigen::Vector3d e2 = trajopt::calcRotationalError(pi_rot_minus.rotation());
      EXPECT_FALSE((e1.norm() < -M_PI));
      EXPECT_FALSE((e1.norm() > M_PI));
      EXPECT_FALSE((e2.norm() < -M_PI));
      EXPECT_FALSE((e2.norm() > M_PI));
      pi_rot_delta = e1 - e2;
      EXPECT_TRUE(pi_rot_delta.norm() > M_PI);  // This is because calcRotationalError breaks down at PI or -PI
    }
  }
}

/** @brief Tests calcRotationalError2 which return angle between [0, 2 * PI]*/
TEST(UtilsUnit, calcRotationalError2)  // NOLINT
{
  auto check_axis = [](const Eigen::Vector3d& axis) {
    return (axis.normalized().isApprox(Eigen::Vector3d::UnitZ(), 1e-6) ||
            axis.normalized().isApprox(-Eigen::Vector3d::UnitZ(), 1e-6));
  };
  Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pi_rot = identity * Eigen::AngleAxisd(3 * M_PI_2, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d rot_err = trajopt::calcRotationalError2(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI_2, 1e-6);
  EXPECT_TRUE(check_axis(rot_err.normalized()));

  pi_rot = identity * Eigen::AngleAxisd(0.0001, Eigen::Vector3d::UnitZ());
  rot_err = trajopt::calcRotationalError2(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), 0.0001, 1e-6);
  EXPECT_TRUE(check_axis(rot_err.normalized()));

  // Test greater than 2 * PI
  pi_rot = identity * Eigen::AngleAxisd(3 * M_PI, Eigen::Vector3d::UnitZ());
  rot_err = trajopt::calcRotationalError2(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI, 1e-6);
  EXPECT_TRUE(check_axis(rot_err.normalized()));

  // Test lessthan than 0
  pi_rot = identity * Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitZ());
  rot_err = trajopt::calcRotationalError2(pi_rot.rotation());
  EXPECT_NEAR(rot_err.norm(), M_PI, 1e-6);
  EXPECT_TRUE(check_axis(rot_err.normalized()));

  // Test for angle between [0, 2 * PI]
  Eigen::Isometry3d pi_rot_plus = identity * Eigen::AngleAxisd(M_PI + 0.001, Eigen::Vector3d::UnitZ());
  Eigen::Isometry3d pi_rot_minus = identity * Eigen::AngleAxisd(M_PI - 0.001, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d pi_rot_delta =
      trajopt::calcRotationalError2(pi_rot_plus.rotation()) - trajopt::calcRotationalError2(pi_rot_minus.rotation());
  EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);

  // Test for angle at 0
  pi_rot_plus = identity * Eigen::AngleAxisd(0.001, Eigen::Vector3d::UnitZ());
  pi_rot_minus = identity * Eigen::AngleAxisd(-0.001, Eigen::Vector3d::UnitZ());
  pi_rot_delta =
      trajopt::calcRotationalError2(pi_rot_plus.rotation()) - trajopt::calcRotationalError2(pi_rot_minus.rotation());
  EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);

  // Test for angle at 2 * PI
  pi_rot_plus = identity * Eigen::AngleAxisd((2 * M_PI) + 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_minus = identity * Eigen::AngleAxisd((2 * M_PI) - 0.001, Eigen::Vector3d::UnitZ());
  pi_rot_delta =
      trajopt::calcRotationalError2(pi_rot_plus.rotation()) - trajopt::calcRotationalError2(pi_rot_minus.rotation());
  EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);

  // Test random axis
  for (int i = 0; i < 100; i++)
  {
    Eigen::Vector3d axis = Eigen::Vector3d::Random().normalized();

    // Avoid M_PI angle because this breaks down
    Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(1000, -5 * M_PI, 5 * M_PI);
    for (Eigen::Index j = 0; j < angles.rows(); j++)
    {
      pi_rot_plus = identity * Eigen::AngleAxisd(angles(j) + 0.001, axis);
      pi_rot_minus = identity * Eigen::AngleAxisd(angles(j) - 0.001, axis);
      Eigen::Vector3d e1 = trajopt::calcRotationalError2(pi_rot_plus.rotation());
      Eigen::Vector3d e2 = trajopt::calcRotationalError2(pi_rot_minus.rotation());
      EXPECT_FALSE((e1.norm() < 0));
      EXPECT_FALSE((e1.norm() > 2 * M_PI));
      EXPECT_FALSE((e2.norm() < 0));
      EXPECT_FALSE((e2.norm() > 2 * M_PI));
      pi_rot_delta = e1 - e2;
      EXPECT_NEAR(pi_rot_delta.norm(), 0.002, 1e-6);
    }
  }
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
