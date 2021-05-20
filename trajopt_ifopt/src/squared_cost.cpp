/**
 * @file squared_cost.cpp
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
#include <trajopt_ifopt/costs/squared_cost.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt
{
SquaredCost::SquaredCost(ifopt::ConstraintSet::Ptr constraint)
  : SquaredCost(std::move(constraint), Eigen::VectorXd::Ones(constraint->GetRows()))
{
}

SquaredCost::SquaredCost(ifopt::ConstraintSet::Ptr constraint, const Eigen::Ref<const Eigen::VectorXd>& weights)
  : CostTerm(constraint->GetName() + "_squared_cost")
  , constraint_(std::move(constraint))
  , n_constraints_(constraint_->GetRows())
  , weights_(weights)
{
  // Calculate targets - Average the upper and lower bounds
  targets_.resize(n_constraints_);
  std::vector<ifopt::Bounds> bounds = constraint_->GetBounds();
  for (Eigen::Index ind = 0; ind < n_constraints_; ind++)
    targets_(ind) = (bounds[static_cast<std::size_t>(ind)].upper_ + bounds[static_cast<std::size_t>(ind)].lower_) / 2.;
}

double SquaredCost::GetCost() const
{
  Eigen::VectorXd values = constraint_->GetValues();
  Eigen::VectorXd error = values - targets_;
  double cost = error.transpose() * weights_.asDiagonal() * error;
  return cost;
}

void SquaredCost::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
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
  Eigen::VectorXd error = constraint_->GetValues() - targets_;
  jac_block = 2 * error.transpose().sparseView() * weights_.asDiagonal() * cnt_jac_block;
}

}  // namespace trajopt
