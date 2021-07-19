/**
 * @file expressions_unit.cpp
 * @brief Unit tests for expressions.h/.cpp
 *
 * @author Levi Armstrong
 * @date July 19, 2021
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
#include <gtest/gtest.h>
#include <trajopt_sqp/expressions.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP
using trajopt_sqp::AffExprs;
using trajopt_sqp::QuadExprs;

TEST(ExpressionsTest, AffExprs)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("ExpressionsTest, AffExprs");
  // f = (x(0) - x(1))^2
  Eigen::Vector2d x(5, 1);
  Eigen::VectorXd e(1);
  e(0) = 16;
  Eigen::Vector2d J(8, -8);

  AffExprs aff_exprs = trajopt_sqp::createAffExprs(e, J.transpose().sparseView(), x);
  EXPECT_NEAR(aff_exprs.constants(0), -16, 1e-8);
  EXPECT_NEAR(aff_exprs.linear_coeffs.coeff(0, 0), 8, 1e-8);
  EXPECT_NEAR(aff_exprs.linear_coeffs.coeff(0, 1), -8, 1e-8);
  EXPECT_NEAR(aff_exprs.values(x)(0), e(0), 1e-8);
}

TEST(ExpressionsTest, QuadExprs)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("ExpressionsTest, QuadExprs");
  // f = (x(0) - x(1))^2
  Eigen::Vector2d x(5, 1);
  Eigen::VectorXd e(1);
  e(0) = 16;
  Eigen::Vector2d J(8, -8);
  Eigen::Matrix2d H;
  H << 2, -2, -2, 2;

  QuadExprs quad_exprs = trajopt_sqp::createQuadExprs(e, J.transpose().sparseView(), { H.sparseView() }, x);
  EXPECT_NEAR(quad_exprs.constants(0), 0, 1e-8);
  EXPECT_NEAR(quad_exprs.linear_coeffs.coeff(0, 0), 0, 1e-8);
  EXPECT_NEAR(quad_exprs.linear_coeffs.coeff(0, 1), 0, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 0), 1, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(1, 1), 1, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 1), -1, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(1, 0), -1, 1e-8);
  EXPECT_NEAR(quad_exprs.values(x)(0), e(0), 1e-8);

  // Because the function is a quadratic it should be an exact fit so check another set of values
  x = Eigen::Vector2d(8, 2);
  EXPECT_NEAR(quad_exprs.values(x)(0), std::pow(x(0) - x(1), 2.0), 1e-8);
}

TEST(ExpressionsTest, squareAffExprs1)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("ExpressionsTest, QuadExprs");
  // This should produce the same results as the QuadExprs unit test

  // f = (x(0) - x(1))
  Eigen::Vector2d x(5, 1);
  Eigen::VectorXd e(1);
  e(0) = x(0) - x(1);
  Eigen::Vector2d J(1, -1);

  AffExprs aff_exprs = trajopt_sqp::createAffExprs(e, J.transpose().sparseView(), x);
  EXPECT_NEAR(aff_exprs.values(x)(0), e(0), 1e-8);

  QuadExprs quad_exprs = trajopt_sqp::squareAffExprs(aff_exprs);

  EXPECT_NEAR(quad_exprs.constants(0), 0, 1e-8);
  EXPECT_NEAR(quad_exprs.linear_coeffs.coeff(0, 0), 0, 1e-8);
  EXPECT_NEAR(quad_exprs.linear_coeffs.coeff(0, 1), 0, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 0), 1, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(1, 1), 1, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 1), -1, 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(1, 0), -1, 1e-8);
  EXPECT_NEAR(quad_exprs.values(x)(0), std::pow(x(0) - x(1), 2.0), 1e-8);

  // Because the function is a quadratic it should be an exact fit so check another set of values
  x = Eigen::Vector2d(8, 2);
  EXPECT_NEAR(quad_exprs.values(x)(0), std::pow(x(0) - x(1), 2.0), 1e-8);
}

TEST(ExpressionsTest, squareAffExprs2)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("ExpressionsTest, squareAffExprs");
  // f = 5 - (x(0) - x(1))
  Eigen::Vector2d x(5, 1);
  Eigen::VectorXd e(1);
  e(0) = 5 - (x(0) - x(1));
  Eigen::Vector2d J(-1, 1);

  AffExprs aff_exprs = trajopt_sqp::createAffExprs(e, J.transpose().sparseView(), x);
  EXPECT_NEAR(aff_exprs.values(x)(0), e(0), 1e-8);

  // The affine expression = (a + b*x + c*y) where x = x(0) and y = x(1)
  // The squared affine expressions = a^2 + 2*a*b*x + 2*a*c*y + 2*b*c*x*y + b^2 * x^2 + c^2 * y^2
  QuadExprs quad_exprs = trajopt_sqp::squareAffExprs(aff_exprs);

  EXPECT_NEAR(quad_exprs.constants(0), std::pow(aff_exprs.constants(0), 2.0), 1e-8);
  EXPECT_NEAR(
      quad_exprs.linear_coeffs.coeff(0, 0), 2 * aff_exprs.constants(0) * aff_exprs.linear_coeffs.coeff(0, 0), 1e-8);
  EXPECT_NEAR(
      quad_exprs.linear_coeffs.coeff(0, 1), 2 * aff_exprs.constants(0) * aff_exprs.linear_coeffs.coeff(0, 1), 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 0), std::pow(aff_exprs.linear_coeffs.coeff(0, 0), 2.0), 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(1, 1), std::pow(aff_exprs.linear_coeffs.coeff(0, 1), 2.0), 1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(0, 1),
              aff_exprs.linear_coeffs.coeff(0, 0) * aff_exprs.linear_coeffs.coeff(0, 1),
              1e-8);
  EXPECT_NEAR(quad_exprs.quadratic_coeffs[0].coeff(1, 0),
              aff_exprs.linear_coeffs.coeff(0, 0) * aff_exprs.linear_coeffs.coeff(0, 1),
              1e-8);
  EXPECT_NEAR(quad_exprs.values(x)(0), std::pow(e(0), 2.0), 1e-8);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
