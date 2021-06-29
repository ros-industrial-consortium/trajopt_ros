/**
 * @file joint_position_constraint.cpp
 * @brief The joint position constraint
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
#include <trajopt_ifopt/constraints/joint_position_constraint.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
JointPosConstraint::JointPosConstraint(const Eigen::VectorXd& targets,
                                       const std::vector<JointPosition::ConstPtr>& position_vars,
                                       const std::string& name)
  : ifopt::ConstraintSet(static_cast<int>(targets.size()) * static_cast<int>(position_vars.size()), name)
  , position_vars_(position_vars)
{
  // Check and make sure the targets size aligns with the vars passed in
  for (auto& position_var : position_vars)
  {
    if (targets.size() != position_var->GetRows())
      CONSOLE_BRIDGE_logError("Targets size does not align with variables provided");
  }

  // Set the n_dof and n_vars for convenience
  n_dof_ = targets.size();
  n_vars_ = static_cast<long>(position_vars.size());
  assert(n_dof_ > 0);
  assert(n_vars_ > 0);

  // Set the bounds to the input targets
  std::vector<ifopt::Bounds> bounds(static_cast<size_t>(GetRows()));
  // All of the positions should be exactly at their targets
  for (long j = 0; j < n_vars_; j++)
  {
    for (long i = 0; i < n_dof_; i++)
    {
      bounds[static_cast<size_t>(i + j * n_dof_)] = ifopt::Bounds(targets[i], targets[i]);
    }
  }
  bounds_ = bounds;
}

JointPosConstraint::JointPosConstraint(const std::vector<ifopt::Bounds>& bounds,
                                       const std::vector<JointPosition::ConstPtr>& position_vars,
                                       const std::string& name)
  : ifopt::ConstraintSet(static_cast<int>(bounds.size()) * static_cast<int>(position_vars.size()), name)
  , bounds_(bounds)
  , position_vars_(position_vars)
{
  // Check and make sure the targets size aligns with the vars passed in
  for (auto& position_var : position_vars_)
  {
    if (static_cast<long>(bounds_.size()) != position_var->GetRows())
      CONSOLE_BRIDGE_logError("Bounds size does not align with variables provided");
  }

  // Set the n_dof and n_vars for convenience
  n_dof_ = static_cast<long>(bounds_.size());
  n_vars_ = static_cast<long>(position_vars_.size());
  assert(n_dof_ > 0);
  assert(n_vars_ > 0);
}

Eigen::VectorXd JointPosConstraint::GetValues() const
{
  // Get the correct variables
  Eigen::VectorXd values(static_cast<size_t>(n_dof_ * n_vars_));
  for (auto& position_var : position_vars_)
    values << this->GetVariables()->GetComponent(position_var->GetName())->GetValues();

  return values;
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> JointPosConstraint::GetBounds() const { return bounds_; }

void JointPosConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Loop over all of the variables this constraint uses
  for (long i = 0; i < n_vars_; i++)
  {
    // Only modify the jacobian if this constraint uses var_set
    if (var_set == position_vars_[static_cast<std::size_t>(i)]->GetName())
    {
      // Reserve enough room in the sparse matrix
      jac_block.reserve(Eigen::VectorXd::Constant(n_dof_, 1));

      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        jac_block.coeffRef(i * n_dof_ * 0 + j, j) = 1.0;
      }
    }
  }
}
}  // namespace trajopt_ifopt
