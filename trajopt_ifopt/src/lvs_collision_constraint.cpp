/**
 * @file lvs_collision_constraint.cpp
 * @brief The longested valid segment collision position constraint
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
#include <trajopt_ifopt/constraints/lvs_collision_constraint.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_kinematics/core/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt
{
LVSCollisionConstraintIfopt::LVSCollisionConstraintIfopt(LVSCollisionEvaluator::Ptr collision_evaluator,
                                                         JointPosition::ConstPtr position_var0,
                                                         JointPosition::ConstPtr position_var1,
                                                         const std::string& name)
  : ifopt::ConstraintSet(1, name)
  , position_var0_(std::move(position_var0))
  , position_var1_(std::move(position_var1))
  , collision_evaluator_(std::move(collision_evaluator))
{
  // Set n_dof_ for convenience
  n_dof_ = position_var0_->GetRows();
  assert(n_dof_ > 0);
  assert(position_var0_->GetRows() == position_var1_->GetRows());

  bounds_ = std::vector<ifopt::Bounds>(1, ifopt::BoundSmallerZero);
}

Eigen::VectorXd LVSCollisionConstraintIfopt::GetValues() const
{
  // Get current joint values
  Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_var0_->GetName())->GetValues();
  Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_var1_->GetName())->GetValues();

  return CalcValues(joint_vals0, joint_vals1);
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> LVSCollisionConstraintIfopt::GetBounds() const { return bounds_; }

void LVSCollisionConstraintIfopt::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == position_var0_->GetName() &&
      collision_evaluator_->GetEvaluatorType() != LVSCollisionEvaluatorType::START_FIXED_END_FREE)
  {
    // Get current joint values
    VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_var0_->GetName())->GetValues();
    VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_var1_->GetName())->GetValues();

    if (collision_evaluator_->GetEvaluatorType() == LVSCollisionEvaluatorType::START_FREE_END_FREE)
      CalcJacobianBlockBothFree(jac_block, joint_vals0, joint_vals1, false);
    else
      CalcJacobianBlockStartFree(jac_block, joint_vals0, joint_vals1);
  }
  else if (var_set == position_var1_->GetName() &&
           collision_evaluator_->GetEvaluatorType() != LVSCollisionEvaluatorType::START_FREE_END_FIXED)
  {
    // Get current joint values
    VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_var0_->GetName())->GetValues();
    VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_var1_->GetName())->GetValues();

    if (collision_evaluator_->GetEvaluatorType() == LVSCollisionEvaluatorType::START_FREE_END_FREE)
      CalcJacobianBlockBothFree(jac_block, joint_vals0, joint_vals1, true);
    else
      CalcJacobianBlockEndFree(jac_block, joint_vals0, joint_vals1);
  }
}

Eigen::VectorXd LVSCollisionConstraintIfopt::CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals0,
                                                        const Eigen::Ref<const Eigen::VectorXd>& joint_vals1) const
{
  Eigen::VectorXd err = Eigen::VectorXd::Zero(1);

  // Check the collisions
  tesseract_collision::ContactResultVector dist_results;
  collision_evaluator_->CalcCollisions(joint_vals0, joint_vals1, dist_results);

  for (tesseract_collision::ContactResult& dist_result : dist_results)
  {
    // Contains the contact distance threshold and coefficient for the given link pair
    double dist = collision_evaluator_->GetCollisionConfig().collision_margin_data.getPairCollisionMargin(
        dist_result.link_names[0], dist_result.link_names[1]);
    double coeff = collision_evaluator_->GetCollisionConfig().collision_coeff_data.getPairCollisionCoeff(
        dist_result.link_names[0], dist_result.link_names[1]);
    // distance will be distance from threshold with negative being greater (further) than
    // the threshold times the coeff
    err[0] += std::max<double>(((dist - dist_result.distance) * coeff), 0.);
  }
  return err;
}

void LVSCollisionConstraintIfopt::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 1);
  bounds_ = bounds;
}

void LVSCollisionConstraintIfopt::CalcJacobianBlockStartFree(Jacobian& jac_block,
                                                             const Eigen::Ref<const Eigen::VectorXd>& joint_vals0,
                                                             const Eigen::Ref<const Eigen::VectorXd>& joint_vals1) const
{
  // Reserve enough room in the sparse matrix
  jac_block.reserve(n_dof_);

  // Calculate collisions
  tesseract_collision::ContactResultVector dist_results;
  collision_evaluator_->CalcCollisions(joint_vals0, joint_vals1, dist_results);

  // Get gradients for all contacts
  std::vector<trajopt::GradientResults> grad_results;
  grad_results.reserve(dist_results.size());
  for (tesseract_collision::ContactResult& dist_result : dist_results)
    grad_results.push_back(collision_evaluator_->GetGradient(joint_vals0, joint_vals1, dist_result, false));

  // Convert GradientResults to jacobian
  int idx = 0;
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(n_dof_);
  for (auto& grad : grad_results)
  {
    if (grad.gradients[0].has_gradient)
      grad_vec += grad.gradients[0].scale * grad.gradients[0].gradient;
    if (grad.gradients[1].has_gradient)
      grad_vec += grad.gradients[1].scale * grad.gradients[1].gradient;
    idx++;
  }

  // This does work but could be faster
  for (int j = 0; j < n_dof_; j++)
  {
    // Collision is 1 x n_dof
    jac_block.coeffRef(0, j) = -1 * grad_vec[j];
  }
}

void LVSCollisionConstraintIfopt::CalcJacobianBlockEndFree(Jacobian& jac_block,
                                                           const Eigen::Ref<const Eigen::VectorXd>& joint_vals0,
                                                           const Eigen::Ref<const Eigen::VectorXd>& joint_vals1) const
{
  // Reserve enough room in the sparse matrix
  jac_block.reserve(n_dof_);

  // Calculate collisions
  tesseract_collision::ContactResultVector dist_results;
  collision_evaluator_->CalcCollisions(joint_vals0, joint_vals1, dist_results);

  // Get gradients for all contacts
  std::vector<trajopt::GradientResults> grad_results;
  grad_results.reserve(dist_results.size());
  for (tesseract_collision::ContactResult& dist_result : dist_results)
    grad_results.push_back(collision_evaluator_->GetGradient(joint_vals0, joint_vals1, dist_result, true));

  // Convert GradientResults to jacobian
  int idx = 0;
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(n_dof_);
  for (auto& grad : grad_results)
  {
    if (grad.gradients[0].has_gradient)
      grad_vec += grad.gradients[0].scale * grad.gradients[0].gradient;
    if (grad.gradients[1].has_gradient)
      grad_vec += grad.gradients[1].scale * grad.gradients[1].gradient;
    idx++;
  }

  // This does work but could be faster
  for (int j = 0; j < n_dof_; j++)
  {
    // Collision is 1 x n_dof
    jac_block.coeffRef(0, j) = -1 * grad_vec[j];
  }
}

void LVSCollisionConstraintIfopt::CalcJacobianBlockBothFree(Jacobian& jac_block,
                                                            const Eigen::Ref<const Eigen::VectorXd>& joint_vals0,
                                                            const Eigen::Ref<const Eigen::VectorXd>& joint_vals1,
                                                            bool isTimestep1) const
{
  // Reserve enough room in the sparse matrix
  jac_block.reserve(n_dof_);

  // Calculate collisions
  tesseract_collision::ContactResultVector dist_results;
  collision_evaluator_->CalcCollisions(joint_vals0, joint_vals1, dist_results);

  // Get gradients for all contacts
  std::vector<trajopt::GradientResults> grad_results;
  grad_results.reserve(dist_results.size());
  for (tesseract_collision::ContactResult& dist_result : dist_results)
    grad_results.push_back(collision_evaluator_->GetGradient(joint_vals0, joint_vals1, dist_result, isTimestep1));

  // Convert GradientResults to jacobian
  int idx = 0;
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(n_dof_);
  for (auto& grad : grad_results)
  {
    if (grad.gradients[0].has_gradient)
      grad_vec += grad.gradients[0].gradient;
    if (grad.gradients[1].has_gradient)
      grad_vec += grad.gradients[1].gradient;
    idx++;
  }

  // This does work but could be faster
  for (int j = 0; j < n_dof_; j++)
  {
    // Collision is 1 x n_dof
    jac_block.coeffRef(0, j) = -1 * grad_vec[j];
  }
}

LVSCollisionEvaluator::Ptr LVSCollisionConstraintIfopt::GetCollisionEvaluator() const { return collision_evaluator_; }

}  // namespace trajopt
