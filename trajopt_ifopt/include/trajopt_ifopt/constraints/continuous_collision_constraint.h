/**
 * @file continuous_collision_constraint.h
 * @brief The continuous collision position constraint
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
#ifndef TRAJOPT_IFOPT_CONTINUOUS_COLLISION_CONSTRAINT_H
#define TRAJOPT_IFOPT_CONTINUOUS_COLLISION_CONSTRAINT_H
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <ifopt/constraint_set.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/continuous_collision_evaluators.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

namespace trajopt
{
class ContinuousCollisionConstraintIfopt : public ifopt::ConstraintSet
{
public:
  using Ptr = std::shared_ptr<ContinuousCollisionConstraintIfopt>;
  using ConstPtr = std::shared_ptr<const ContinuousCollisionConstraintIfopt>;

  /**
   * @brief ContinuousCollisionConstraintIfopt
   * @param collision_evaluator The continuous collision evaluator to use
   * @param gradient_method The gradient method to use for combining all gradients into a single gradient
   * @param position_vars The position vars associated with this constraint. The middle position var is the var set for
   * the constraint. The pre and post are supporting variables for calculating the constraint error and gradient
   *
   *  There are three conditions of the array that are acceptable:
   *    Case 1: <nullptr, ConstPtr, ConstPtr> This would be usually be the first position in the trajectory where it
   * does not have previous position var set Case 2: <ConstPtr, ConstPtr, nullptr> This would be usually be the last
   * position in the trajectory where it does not have post position var set Case 3: <ConstPtr, ConstPtr, ConstPtr> This
   * would usually be used for all other states in the trajectory because it has both a pre and post position var set
   * @param name
   */
  ContinuousCollisionConstraintIfopt(ContinuousCollisionEvaluator::Ptr collision_evaluator,
                                     GradientCombineMethod gradient_method,
                                     std::array<JointPosition::ConstPtr, 3> position_vars,
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

  /**
   * @brief Calculates the values associated with the constraint
   * @details These are exposed for unit tests and debugging.
   */
  Eigen::VectorXd CalcValuesPost(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_vals_post) const;

  Eigen::VectorXd CalcValuesCent(const Eigen::Ref<const Eigen::VectorXd>& joint_vals_prev,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_vals_post) const;

  Eigen::VectorXd CalcValuesPrev(const Eigen::Ref<const Eigen::VectorXd>& joint_vals_prev,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const;

  /**
   * @brief Sets the bounds on the collision distance
   * @param bounds New bounds that will be set. Should be size 1
   */
  void SetBounds(const std::vector<ifopt::Bounds>& bounds);

  /**
   * @brief Get the collision evaluator. This exposed for plotter callbacks
   * @return The collision evaluator
   */
  ContinuousCollisionEvaluator::Ptr GetCollisionEvaluator() const;

private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_;

  /** @brief Bounds on the constraint value. Default: std::vector<Bounds>(1, ifopt::BoundSmallerZero) */
  std::vector<ifopt::Bounds> bounds_;

  /**
   * @brief Pointers to the vars used by this constraint.
   * Do not access them directly. Instead use this->GetVariables()->GetComponent(position_var->GetName())->GetValues()
   */
  std::array<JointPosition::ConstPtr, 3> position_vars_;

  ContinuousCollisionEvaluator::Ptr collision_evaluator_;
  GradientCombineMethod gradient_method_;

  /**
   * @brief This is used internally to change how the values and gradient is calculated based on the position var set
   * array
   * @details
   *    mode_ = 0, is central
   *    mode_ = 1, is post
   *    mode_ = 2, is previous
   */
  int mode_{ -1 };

  void CalcJacobianBlockPrev(Jacobian& jac_block,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_vals_prev,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const;

  void CalcJacobianBlockPost(Jacobian& jac_block,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_vals_post) const;

  void CalcJacobianBlockCent(Jacobian& jac_block,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_vals_prev,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_vals_post) const;
};
};  // namespace trajopt

#endif  // TRAJOPT_IFOPT_CONTINUOUS_COLLISION_CONSTRAINT_H
