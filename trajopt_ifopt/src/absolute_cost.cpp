/**
 * @file absolute_cost.cpp
 * @brief The absolute cost. Converts a constraint into a cost.
 *
 * @author Levi Armstrong
 * @date May 20, 20201
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
#include <trajopt_ifopt/costs/absolute_cost.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt
{
AbsoluteCost::AbsoluteCost(ifopt::ConstraintSet::Ptr constraint)
  : AbsoluteCost(std::move(constraint), Eigen::VectorXd::Ones(constraint->GetRows()))
{
}

AbsoluteCost::AbsoluteCost(ifopt::ConstraintSet::Ptr constraint, const Eigen::Ref<const Eigen::VectorXd>& weights)
  : CostTerm(constraint->GetName() + "_absolute_cost")
  , constraint_(std::move(constraint))
  , n_constraints_(constraint_->GetRows())
  , weights_(weights)
{
}

double AbsoluteCost::GetCost() const
{
  // This takes the absolute value of the errors
  Eigen::VectorXd error = calcBoundsViolations(constraint_->GetValues(), constraint_->GetBounds());
  double cost = weights_.transpose() * error;
  return cost;
}

void AbsoluteCost::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Get a Jacobian block the size necessary for the constraint
  Jacobian cnt_jac_block;
  int var_size = 0;
  for (const auto& vars : GetVariables()->GetComponents())
  {
    if (vars->GetName() == var_set)
      var_size = vars->GetRows();
  }
  assert(var_size > 0);
  cnt_jac_block.resize(constraint_->GetRows(), var_size);

  // Get the Jacobian Block from the constraint
  constraint_->FillJacobianBlock(var_set, cnt_jac_block);

  // Apply the chain rule. See doxygen for this class
  Eigen::ArrayXd error = calcBoundsErrors(constraint_->GetValues(), constraint_->GetBounds());
  Eigen::ArrayXd w_error = error * weights_.array();
  Eigen::VectorXd coeff = w_error / error.abs();

  jac_block = coeff.sparseView() * cnt_jac_block;
}

}  // namespace trajopt
