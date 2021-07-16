/**
 * @file inverse_kinematics_constraint.cpp
 * @brief The cartesian position constraint
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
#include <trajopt_ifopt/constraints/inverse_kinematics_constraint.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_kinematics/core/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
InverseKinematicsConstraint::InverseKinematicsConstraint(const Eigen::Isometry3d& target_pose,
                                                         InverseKinematicsInfo::ConstPtr kinematic_info,
                                                         JointPosition::ConstPtr constraint_var,
                                                         JointPosition::ConstPtr seed_var,
                                                         const std::string& name)
  : ifopt::ConstraintSet(constraint_var->GetRows(), name)
  , constraint_var_(std::move(constraint_var))
  , seed_var_(std::move(seed_var))
  , target_pose_(target_pose)
  , kinematic_info_(std::move(kinematic_info))
{
  // Set the n_dof and n_vars for convenience
  n_dof_ = constraint_var_->GetRows();
  assert(n_dof_ > 0);
  if (static_cast<std::size_t>(constraint_var_->GetRows()) != kinematic_info_->inverse_kinematics->numJoints())
    CONSOLE_BRIDGE_logError("Inverse kinematics has a different number of joints than the given variable set");

  bounds_ = std::vector<ifopt::Bounds>(static_cast<std::size_t>(n_dof_), ifopt::BoundZero);
}

Eigen::VectorXd
InverseKinematicsConstraint::CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                        const Eigen::Ref<const Eigen::VectorXd>& seed_joint_position) const
{
  // Get joint position using IK and the seed variable
  tesseract_kinematics::IKSolutions target_joint_position =
      kinematic_info_->inverse_kinematics->calcInvKin(target_pose_, seed_joint_position);
  assert(!target_joint_position.empty());

  // Calculate joint error
  Eigen::VectorXd error = Eigen::VectorXd::Zero(joint_vals.rows());
  double error_norm = std::numeric_limits<double>::max();
  for (const auto& sol : target_joint_position)
  {
    Eigen::VectorXd cur_error = sol - joint_vals;
    double cur_error_norm = cur_error.norm();
    if (cur_error_norm < error_norm)
    {
      error_norm = cur_error_norm;
      error = cur_error;
    }
  }
  return error;
}

Eigen::VectorXd InverseKinematicsConstraint::GetValues() const
{
  // Get the two variables
  Eigen::VectorXd seed_joint_position = this->GetVariables()->GetComponent(seed_var_->GetName())->GetValues();
  Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(constraint_var_->GetName())->GetValues();

  return CalcValues(joint_vals, seed_joint_position);
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> InverseKinematicsConstraint::GetBounds() const { return bounds_; }

void InverseKinematicsConstraint::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  if (bounds.size() != static_cast<std::size_t>(n_dof_))
    CONSOLE_BRIDGE_logError("Bounds is incorrect size. It is %d when it should be %d", bounds.size(), n_dof_);

  bounds_ = bounds;
}

void InverseKinematicsConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == constraint_var_->GetName())
  {
    // Reserve enough room in the sparse matrix
    jac_block.reserve(n_dof_);

    for (int j = 0; j < n_dof_; j++)
    {
      // err = target - x =? derr/dx = -1
      jac_block.coeffRef(j, j) = -1;
    }
  }
}

void InverseKinematicsConstraint::SetTargetPose(const Eigen::Isometry3d& target_pose) { target_pose_ = target_pose; }
}  // namespace trajopt_ifopt
