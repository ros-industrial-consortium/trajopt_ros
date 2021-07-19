/**
 * @file squared_cost.h
 * @brief The squared cost. Converts a constraint into a cost.
 *
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
#ifndef TRAJOPT_IFOPT_SQUARED_COST_H
#define TRAJOPT_IFOPT_SQUARED_COST_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/cost_term.h>

#include <Eigen/Eigen>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
/**
 * @brief This class converts a constraint to a cost with a sum squared error
 *
 * IFOPT costs differ from constraints in 2 main ways.
 * 1) Constraints have multiple rows where a cost is simply a value (the sum of all of the costs)
 * 2) Constraints have bounds that they must be within. Costs should simply be as close to 0 as possible
 *
 * This class converts the bounds to a target and then sets the value to be the weighted squared sum of the distances
 * from that target. The result is that the value (GetCost) is a single double and the jacobian (GetJacobian) is a
 * single row (1 x n_vars)
 *
 * Given a constraint: g(x) with some bounds: upper_ and lower_
 *
 * The target of the cost is set to the closest bound:
 *
 *     target = to the closest bound
 *
 * and
 *
 *     error(x) = g(x) - target
 *
 * Let
 *
 *     derror(x)/dx = dg(x)/dx = J(x)
 *
 * The squared cost is defined as
 *
 *     cost(x) = error.transpose() * W * error
 *
 * where W is the diagonal matrix of the cost weights
 *
 * Then
 *
 *     dcost(x)/dx = 2 * error.transpose() * W * J(x)
 *
 */
class SquaredCost : public ifopt::CostTerm
{
public:
  using Ptr = std::shared_ptr<SquaredCost>;
  using ConstPtr = std::shared_ptr<const SquaredCost>;

  /**
   * @brief Constructs a CostTerm that converts a constraint into a cost with a sum squared error
   * @param constraint Input constraint to be converted to a cost
   */
  SquaredCost(ifopt::ConstraintSet::Ptr constraint);
  /**
   * @brief Constructs a CostTerm that converts a constraint into a cost with a weighted sum squared error
   * @param constraint Input constraint to be converted to a cost
   * @param weights Weights applied to the constraints. Length should be n_constraints
   */
  SquaredCost(ifopt::ConstraintSet::Ptr constraint, const Eigen::Ref<const Eigen::VectorXd>& weights);

  double GetCost() const override;

  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

private:
  /** @brief Constraint being converted to a cost */
  std::shared_ptr<const ConstraintSet> constraint_;

  /** @brief Size of the input constraint */
  long n_constraints_;

  /** @brief Vector of weights. Default: Eigen::VectorXd::Ones(n_constraints) */
  Eigen::VectorXd weights_;
};

}  // namespace trajopt_ifopt
#endif
