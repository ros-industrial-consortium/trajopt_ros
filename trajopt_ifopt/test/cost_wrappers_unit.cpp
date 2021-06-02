/**
 * @file cost_wrappers_unit.cpp
 * @brief Test the cost wrappers unit
 *
 * @author Levi Armstrong
 * @date May 20, 2021
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
#include <Eigen/Eigen>
#include <ifopt/problem.h>
#include <ifopt/constraint_set.h>
#include <gtest/gtest.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/costs/absolute_cost.h>
#include <trajopt_ifopt/costs/squared_cost.h>

/**
 * @brief The SimpleTestConstraint class
 * @details
 *    y(x) = x^2 + 4x + 3
 *   dy(x) = 2x + 4
 */
class SimpleTestConstraint : public ifopt::ConstraintSet
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimpleTestConstraint(trajopt::JointPosition::ConstPtr position_var, const std::string& name = "SimpleTestConstraint")
    : ifopt::ConstraintSet(1, name), position_var_(std::move(position_var))
  {
    bounds_ = std::vector<ifopt::Bounds>(1, ifopt::BoundZero);
  }

  Eigen::VectorXd GetValues() const final
  {
    Eigen::VectorXd output(1);
    Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
    output(0) = std::pow(joint_vals(0), 2) + 4 * joint_vals(0) + 3;
    return output;
  }

  std::vector<ifopt::Bounds> GetBounds() const final { return bounds_; }

  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const final
  {
    // Only modify the jacobian if this constraint uses var_set
    if (var_set == position_var_->GetName())
    {
      // dy = 2x + 4;
      Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();

      // Reserve enough room in the sparse matrix
      jac_block.reserve(1);
      jac_block.coeffRef(0, 0) = 2 * joint_vals(0) + 4;
    }
  }

private:
  /** @brief Bounds on the positions of each joint */
  std::vector<ifopt::Bounds> bounds_;

  /** @brief Pointers to the vars used by this constraint.
   *
   * Do not access them directly. Instead use this->GetVariables()->GetComponent(position_var->GetName())->GetValues()*/
  trajopt::JointPosition::ConstPtr position_var_;
};

/** @brief Tests that GetValues and FillJacobianBlock return the correct values */
TEST(CostWrapperUnit, SquaredCost)  // NOLINT
{
  // 2) Create the problem
  ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<trajopt::JointPosition::ConstPtr> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    Eigen::VectorXd pos(1);
    pos << -4;
    positions.push_back(pos);
    std::vector<std::string> var_names = { "x" };
    auto var = std::make_shared<trajopt::JointPosition>(pos, var_names);
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }

  // 4) add cost
  auto cnt = std::make_shared<SimpleTestConstraint>(vars[0]);
  cnt->LinkWithVariables(nlp.GetOptVariables());
  auto cost = std::make_shared<trajopt::SquaredCost>(cnt);
  nlp.AddCostSet(cost);

  auto exact_jac = nlp.EvaluateCostFunctionGradient(positions[0].data());
  auto numerical_jac = nlp.EvaluateCostFunctionGradient(positions[0].data(), true, 1e-8);

  EXPECT_NEAR(exact_jac(0, 0), numerical_jac(0, 0), 1e-6);

  // Squared cost jacobian = 2 * y(x) * dy(x)
  //   y(x) = x^2 + 4x + 3
  //   dy(x) = 2x + 4
  double x = positions[0](0);
  double jac = 2 * (std::pow(x, 2) + 4 * x + 3) * (2 * x + 4);
  EXPECT_NEAR(exact_jac(0, 0), jac, 1e-6);
  EXPECT_NEAR(numerical_jac(0, 0), jac, 1e-6);
}

/** @brief Tests that GetValues and FillJacobianBlock return the correct values */
TEST(CostWrapperUnit, WeightedSquaredCost)  // NOLINT
{
  // 2) Create the problem
  ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<trajopt::JointPosition::ConstPtr> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    Eigen::VectorXd pos(1);
    pos << -4;
    positions.push_back(pos);
    std::vector<std::string> var_names = { "x" };
    auto var = std::make_shared<trajopt::JointPosition>(pos, var_names);
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }

  // 4) add cost
  Eigen::VectorXd weights(1);
  weights(0) = 2;
  auto cnt = std::make_shared<SimpleTestConstraint>(vars[0]);
  cnt->LinkWithVariables(nlp.GetOptVariables());
  auto cost = std::make_shared<trajopt::SquaredCost>(cnt, weights);
  nlp.AddCostSet(cost);

  auto exact_jac = nlp.EvaluateCostFunctionGradient(positions[0].data());
  auto numerical_jac = nlp.EvaluateCostFunctionGradient(positions[0].data(), true, 1e-8);

  EXPECT_NEAR(exact_jac(0, 0), numerical_jac(0, 0), 1e-6);

  // Squared cost jacobian = 2 * y(x) * dy(x)
  //   y(x) = x^2 + 4x + 3
  //   dy(x) = 2x + 4
  double x = positions[0](0);
  double jac = 2 * weights(0) * (std::pow(x, 2) + 4 * x + 3) * (2 * x + 4);
  EXPECT_NEAR(exact_jac(0, 0), jac, 1e-6);
  EXPECT_NEAR(numerical_jac(0, 0), jac, 1e-6);
}

/** @brief Tests that GetValues and FillJacobianBlock return the correct values */
TEST(CostWrapperUnit, AbsoluteCost)  // NOLINT
{
  // 2) Create the problem
  ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<trajopt::JointPosition::ConstPtr> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    Eigen::VectorXd pos(1);
    pos << -4;
    positions.push_back(pos);
    std::vector<std::string> var_names = { "x" };
    auto var = std::make_shared<trajopt::JointPosition>(pos, var_names);
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }

  // 4) add cost
  auto cnt = std::make_shared<SimpleTestConstraint>(vars[0]);
  cnt->LinkWithVariables(nlp.GetOptVariables());
  auto cost = std::make_shared<trajopt::AbsoluteCost>(cnt);
  nlp.AddCostSet(cost);

  auto exact_jac = nlp.EvaluateCostFunctionGradient(positions[0].data());
  auto numerical_jac = nlp.EvaluateCostFunctionGradient(positions[0].data(), true, 1e-8);

  EXPECT_NEAR(exact_jac(0, 0), numerical_jac(0, 0), 1e-6);

  // Absolute cost jacobian = (y(x) / |y(x)|) * dy(x)
  //   y(x) = x^2 + 4x + 3
  //   dy(x) = 2x + 4
  double x = positions[0](0);
  double jac = ((std::pow(x, 2) + 4 * x + 3) / std::abs(std::pow(x, 2) + 4 * x + 3)) * (2 * x + 4);
  EXPECT_NEAR(exact_jac(0, 0), jac, 1e-6);
  EXPECT_NEAR(numerical_jac(0, 0), jac, 1e-6);
}

/** @brief Tests that GetValues and FillJacobianBlock return the correct values */
TEST(CostWrapperUnit, WeightedAbsoluteCost)  // NOLINT
{
  // 2) Create the problem
  ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<trajopt::JointPosition::ConstPtr> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    Eigen::VectorXd pos(1);
    pos << -4;
    positions.push_back(pos);
    std::vector<std::string> var_names = { "x" };
    auto var = std::make_shared<trajopt::JointPosition>(pos, var_names);
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }

  // 4) add cost
  Eigen::VectorXd weights(1);
  weights(0) = 2;
  auto cnt = std::make_shared<SimpleTestConstraint>(vars[0]);
  cnt->LinkWithVariables(nlp.GetOptVariables());
  auto cost = std::make_shared<trajopt::AbsoluteCost>(cnt, weights);
  nlp.AddCostSet(cost);

  auto exact_jac = nlp.EvaluateCostFunctionGradient(positions[0].data());
  auto numerical_jac = nlp.EvaluateCostFunctionGradient(positions[0].data(), true, 1e-8);

  EXPECT_NEAR(exact_jac(0, 0), numerical_jac(0, 0), 1e-6);

  // Absolute cost jacobian = (y(x) / |y(x)|) * dy(x)
  //   y(x) = x^2 + 4x + 3
  //   dy(x) = 2x + 4
  double x = positions[0](0);
  double jac = (weights(0) * (std::pow(x, 2) + 4 * x + 3) / std::abs(std::pow(x, 2) + 4 * x + 3)) * (2 * x + 4);
  EXPECT_NEAR(exact_jac(0, 0), jac, 1e-6);
  EXPECT_NEAR(numerical_jac(0, 0), jac, 1e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
