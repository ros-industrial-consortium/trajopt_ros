/**
 * @file lvs_collision_constraint.h
 * @brief The longest valid segment collision position constraint
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
#ifndef TRAJOPT_IFOPT_LVS_COLLISION_CONSTRAINT_H
#define TRAJOPT_IFOPT_LVS_COLLISION_CONSTRAINT_H
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <ifopt/constraint_set.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/lvs_collision_evaluators.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

namespace trajopt
{
class LVSCollisionConstraintIfopt : public ifopt::ConstraintSet
{
public:
  using Ptr = std::shared_ptr<LVSCollisionConstraintIfopt>;
  using ConstPtr = std::shared_ptr<const LVSCollisionConstraintIfopt>;

  LVSCollisionConstraintIfopt(LVSCollisionEvaluator::Ptr collision_evaluator,
                              JointPosition::ConstPtr position_var0,
                              JointPosition::ConstPtr position_var1,
                              const std::string& name = "LVSCollision");

  /**
   * @brief Returns the values associated with the constraint.
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
   * @param jac_block Block of the overall jacobian associated with these constraints and the var_set variable
   */
  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

  /** @brief Calculates the values associated with the constraint */
  Eigen::VectorXd CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals0,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_vals1) const;
  /**
   * @brief Sets the bounds on the collision distance
   * @param bounds New bounds that will be set. Should be size 1
   */
  void SetBounds(const std::vector<ifopt::Bounds>& bounds);

  /**
   * @brief Get the collision evaluator. This exposed for plotter callbacks
   * @return The collision evaluator
   */
  LVSCollisionEvaluator::Ptr GetCollisionEvaluator() const;

private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_;

  /** @brief Bounds on the constraint value. Default: std::vector<Bounds>(1, ifopt::BoundSmallerZero) */
  std::vector<ifopt::Bounds> bounds_;

  /**
   * @brief Pointers to the vars used by this constraint.
   * Do not access them directly. Instead use this->GetVariables()->GetComponent(position_var->GetName())->GetValues()
   */
  JointPosition::ConstPtr position_var0_;
  JointPosition::ConstPtr position_var1_;

  LVSCollisionEvaluator::Ptr collision_evaluator_;

  void CalcJacobianBlockStartFree(Jacobian& jac_block,
                                  const Eigen::Ref<const Eigen::VectorXd>& joint_vals0,
                                  const Eigen::Ref<const Eigen::VectorXd>& joint_vals1) const;

  void CalcJacobianBlockEndFree(Jacobian& jac_block,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_vals0,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_vals1) const;

  void CalcJacobianBlockBothFree(Jacobian& jac_block,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_vals0,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_vals1,
                                 bool isTimestep1) const;
};
};  // namespace trajopt

#endif  // TRAJOPT_IFOPT_LVS_COLLISION_CONSTRAINT_H
