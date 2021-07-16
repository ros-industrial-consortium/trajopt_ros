/**
 * @file joint_position_constraint.h
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
#ifndef TRAJOPT_IFOPT_JOINT_POSITION_CONSTRAINT_H
#define TRAJOPT_IFOPT_JOINT_POSITION_CONSTRAINT_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/constraint_set.h>
#include <Eigen/Eigen>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

namespace trajopt_ifopt
{
/**
 * @brief This creates a joint position constraint. Allows bounds to be set on a joint position
 */
class JointPosConstraint : public ifopt::ConstraintSet
{
public:
  using Ptr = std::shared_ptr<JointPosConstraint>;
  using ConstPtr = std::shared_ptr<const JointPosConstraint>;

  /**
   * @brief JointPosConstraint
   * @param targets Target joint position (length should be n_dof). Upper and lower bounds are set to this value
   * @param position_vars Variables to which this constraint is applied. Note that all variables should have the same
   * number of components (joint DOF)
   * @param name Name of the constraint
   */
  JointPosConstraint(const Eigen::VectorXd& targets,
                     const std::vector<JointPosition::ConstPtr>& position_vars,
                     const std::string& name = "JointPos");

  /**
   * @brief JointPosConstraint
   * @param bounds Bounds on target joint position (length should be n_dof)
   * @param position_vars Variables to which this constraint is applied
   * @param name Name of the constraint
   */
  JointPosConstraint(const std::vector<ifopt::Bounds>& bounds,
                     const std::vector<JointPosition::ConstPtr>& position_vars,
                     const std::string& name = "JointPos");

  /**
   * @brief Returns the values associated with the constraint. In this case that is the concatenated joint values
   * associated with each of the joint positions should be n_dof_ * n_vars_ long
   * @return
   */
  Eigen::VectorXd GetValues() const override;

  /**
   * @brief  Returns the "bounds" of this constraint. How these are enforced is up to the solver
   * @return Returns the "bounds" of this constraint
   */
  std::vector<ifopt::Bounds> GetBounds() const override;

  /**
   * @brief Fills the jacobian block associated with the given var_set.
   * @param var_set Name of the var_set to which the jac_block is associated
   * @param jac_block Block of the overal jacobian associated with these constraints and the var_set variable
   */
  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_;
  /** @brief The number of JointPositions passed in */
  long n_vars_;

  /** @brief Bounds on the positions of each joint */
  std::vector<ifopt::Bounds> bounds_;

  /** @brief Pointers to the vars used by this constraint.
   *
   * Do not access them directly. Instead use this->GetVariables()->GetComponent(position_var->GetName())->GetValues()*/
  std::vector<JointPosition::ConstPtr> position_vars_;
};
};  // namespace trajopt_ifopt
#endif
