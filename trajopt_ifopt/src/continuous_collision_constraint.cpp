/**
 * @file continuous_collision_constraint.cpp
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
#include <trajopt_ifopt/constraints/continuous_collision_constraint.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_kinematics/core/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision_utils.h>

namespace trajopt
{
ContinuousCollisionConstraintIfopt::ContinuousCollisionConstraintIfopt(
    ContinuousCollisionEvaluator::Ptr collision_evaluator,
    GradientCombineMethod gradient_method,
    JointPosition::ConstPtr position_var0,
    JointPosition::ConstPtr position_var1,
    const std::string& name)
  : ifopt::ConstraintSet(1, name)
  , position_var0_(std::move(position_var0))
  , position_var1_(std::move(position_var1))
  , collision_evaluator_(std::move(collision_evaluator))
  , gradient_method_(gradient_method)
{
  // Set n_dof_ for convenience
  n_dof_ = position_var0_->GetRows();
  assert(n_dof_ > 0);
  assert(position_var0_->GetRows() == position_var1_->GetRows());

  bounds_ = std::vector<ifopt::Bounds>(1, ifopt::BoundSmallerZero);
}

Eigen::VectorXd ContinuousCollisionConstraintIfopt::GetValues() const
{
  // Get current joint values
  Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_var0_->GetName())->GetValues();
  Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_var1_->GetName())->GetValues();

  return CalcValues(joint_vals0, joint_vals1);
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> ContinuousCollisionConstraintIfopt::GetBounds() const { return bounds_; }

void ContinuousCollisionConstraintIfopt::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == position_var0_->GetName() &&
      collision_evaluator_->GetEvaluatorType() != ContinuousCollisionEvaluatorType::START_FIXED_END_FREE)
  {
    // Get current joint values
    VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_var0_->GetName())->GetValues();
    VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_var1_->GetName())->GetValues();

    if (collision_evaluator_->GetEvaluatorType() == ContinuousCollisionEvaluatorType::START_FREE_END_FREE)
      CalcJacobianBlockBothFree(jac_block, joint_vals0, joint_vals1, false);
    else
      CalcJacobianBlockStartFree(jac_block, joint_vals0, joint_vals1);
  }
  else if (var_set == position_var1_->GetName() &&
           collision_evaluator_->GetEvaluatorType() != ContinuousCollisionEvaluatorType::START_FREE_END_FIXED)
  {
    // Get current joint values
    VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_var0_->GetName())->GetValues();
    VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_var1_->GetName())->GetValues();

    if (collision_evaluator_->GetEvaluatorType() == ContinuousCollisionEvaluatorType::START_FREE_END_FREE)
      CalcJacobianBlockBothFree(jac_block, joint_vals0, joint_vals1, true);
    else
      CalcJacobianBlockEndFree(jac_block, joint_vals0, joint_vals1);
  }
}

Eigen::VectorXd
ContinuousCollisionConstraintIfopt::CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals0,
                                               const Eigen::Ref<const Eigen::VectorXd>& joint_vals1) const
{
  Eigen::VectorXd err = Eigen::VectorXd::Zero(1);

  // Check the collisions
  tesseract_collision::ContactResultVector dist_results;
  collision_evaluator_->CalcCollisions(joint_vals0, joint_vals1, dist_results);

  if (dist_results.empty())
    return err;

  switch (gradient_method_)
  {
    case GradientCombineMethod::SUM:
    {
      for (tesseract_collision::ContactResult& dist_result : dist_results)
      {
        double dist = collision_evaluator_->GetCollisionConfig().collision_margin_data.getPairCollisionMargin(
            dist_result.link_names[0], dist_result.link_names[1]);
        double coeff = collision_evaluator_->GetCollisionConfig().collision_coeff_data.getPairCollisionCoeff(
            dist_result.link_names[0], dist_result.link_names[1]);
        err[0] += std::max<double>(((dist - dist_result.distance) * coeff), 0.);
      }
      break;
    }
    case GradientCombineMethod::WEIGHTED_SUM:
    {
      for (tesseract_collision::ContactResult& dist_result : dist_results)
      {
        double dist = collision_evaluator_->GetCollisionConfig().collision_margin_data.getPairCollisionMargin(
            dist_result.link_names[0], dist_result.link_names[1]);
        double coeff = collision_evaluator_->GetCollisionConfig().collision_coeff_data.getPairCollisionCoeff(
            dist_result.link_names[0], dist_result.link_names[1]);
        err[0] += std::max<double>(((dist - dist_result.distance) * coeff), 0.);
      }
      break;
    }
    case GradientCombineMethod::AVERAGE:
    {
      for (tesseract_collision::ContactResult& dist_result : dist_results)
      {
        double dist = collision_evaluator_->GetCollisionConfig().collision_margin_data.getPairCollisionMargin(
            dist_result.link_names[0], dist_result.link_names[1]);
        double coeff = collision_evaluator_->GetCollisionConfig().collision_coeff_data.getPairCollisionCoeff(
            dist_result.link_names[0], dist_result.link_names[1]);
        double d = std::max<double>(((dist - dist_result.distance) * coeff), 0.);
        if (d > err[0])
          err[0] = d;
      }
      break;
    }
    case GradientCombineMethod::WEIGHTED_AVERAGE:
    {
      for (tesseract_collision::ContactResult& dist_result : dist_results)
      {
        trajopt::GradientResults result =
            collision_evaluator_->GetGradient(joint_vals0, joint_vals1, dist_result, false);

        double dist = collision_evaluator_->GetCollisionConfig().collision_margin_data.getPairCollisionMargin(
            dist_result.link_names[0], dist_result.link_names[1]);
        double coeff = collision_evaluator_->GetCollisionConfig().collision_coeff_data.getPairCollisionCoeff(
            dist_result.link_names[0], dist_result.link_names[1]);
        double d = std::max<double>(((dist - dist_result.distance) * coeff), 0.);
        if (result.gradients[0].has_gradient)
        {
          double wd = (d * result.gradients[0].scale);
          if (wd > err[0])
            err[0] = wd;
        }

        if (result.gradients[1].has_gradient)
        {
          double wd = (d * result.gradients[1].scale);
          if (wd > err[0])
            err[0] = wd;
        }
      }
      break;
    }
    case GradientCombineMethod::LEAST_SQUARES:
    {
      for (tesseract_collision::ContactResult& dist_result : dist_results)
      {
        double dist = collision_evaluator_->GetCollisionConfig().collision_margin_data.getPairCollisionMargin(
            dist_result.link_names[0], dist_result.link_names[1]);
        double coeff = collision_evaluator_->GetCollisionConfig().collision_coeff_data.getPairCollisionCoeff(
            dist_result.link_names[0], dist_result.link_names[1]);
        err[0] += std::max<double>(((dist - dist_result.distance) * coeff), 0.);
      }
      err[0] = err[0] / static_cast<double>(dist_results.size());
      break;
    }
    case GradientCombineMethod::WEIGHTED_LEAST_SQUARES:
    {
      for (tesseract_collision::ContactResult& dist_result : dist_results)
      {
        double dist = collision_evaluator_->GetCollisionConfig().collision_margin_data.getPairCollisionMargin(
            dist_result.link_names[0], dist_result.link_names[1]);
        double coeff = collision_evaluator_->GetCollisionConfig().collision_coeff_data.getPairCollisionCoeff(
            dist_result.link_names[0], dist_result.link_names[1]);
        err[0] += std::max<double>((std::pow(dist - dist_result.distance, 2) * coeff), 0.);
      }
      err[0] = err[0] / static_cast<double>(dist_results.size());
      break;
    }
  }

  return err;
}

void ContinuousCollisionConstraintIfopt::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 1);
  bounds_ = bounds;
}

void ContinuousCollisionConstraintIfopt::CalcJacobianBlockStartFree(
    Jacobian& jac_block,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals0,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals1) const
{
  // Reserve enough room in the sparse matrix
  jac_block.reserve(n_dof_);

  // Calculate collisions
  tesseract_collision::ContactResultVector dist_results;
  collision_evaluator_->CalcCollisions(joint_vals0, joint_vals1, dist_results);

  // Get gradients for all contacts
  long num_eq{ 0 };
  GradientResultsSet grad_set(dist_results.size());
  for (tesseract_collision::ContactResult& dist_result : dist_results)
  {
    trajopt::GradientResults result = collision_evaluator_->GetGradient(joint_vals0, joint_vals1, dist_result, false);
    num_eq += (result.gradients[0].has_gradient + result.gradients[1].has_gradient);
    grad_set.add(result);
  }

  // Convert GradientResults to jacobian
  Eigen::VectorXd grad_vec;
  switch (gradient_method_)
  {
    case GradientCombineMethod::SUM:
    {
      grad_vec = getSumGradient(grad_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_SUM:
    {
      grad_vec = getSumGradient(grad_set, n_dof_);
      break;
    }
    case GradientCombineMethod::AVERAGE:
    {
      grad_vec = getAvgGradient(grad_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_AVERAGE:
    {
      grad_vec = getWeightedScaledAvgGradient(grad_set, n_dof_);
      break;
    }
    case GradientCombineMethod::LEAST_SQUARES:
    {
      grad_vec = getLeastSquaresGradient(grad_set, n_dof_, num_eq);
      break;
    }
    case GradientCombineMethod::WEIGHTED_LEAST_SQUARES:
    {
      grad_vec = getWeightedLeastSquaresGradient(grad_set, n_dof_, num_eq);
      // grad_vec = getWeightedLeastSquaresGradient2(grad_set, n_dof_, num_eq);
      break;
    }
  }

  // This does work but could be faster
  for (int j = 0; j < n_dof_; j++)
  {
    // Collision is 1 x n_dof
    jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
  }
}

void ContinuousCollisionConstraintIfopt::CalcJacobianBlockEndFree(
    Jacobian& jac_block,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals0,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals1) const
{
  // Reserve enough room in the sparse matrix
  jac_block.reserve(n_dof_);

  // Calculate collisions
  tesseract_collision::ContactResultVector dist_results;
  collision_evaluator_->CalcCollisions(joint_vals0, joint_vals1, dist_results);

  // Get gradients for all contacts
  long num_eq{ 0 };
  GradientResultsSet grad_set(dist_results.size());
  for (tesseract_collision::ContactResult& dist_result : dist_results)
  {
    trajopt::GradientResults result = collision_evaluator_->GetGradient(joint_vals0, joint_vals1, dist_result, false);
    num_eq += (result.gradients[0].has_gradient + result.gradients[1].has_gradient);
    grad_set.add(result);
  }

  // Convert GradientResults to jacobian
  Eigen::VectorXd grad_vec;
  switch (gradient_method_)
  {
    case GradientCombineMethod::SUM:
    {
      grad_vec = getSumGradient(grad_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_SUM:
    {
      grad_vec = getSumGradient(grad_set, n_dof_);
      break;
    }
    case GradientCombineMethod::AVERAGE:
    {
      grad_vec = getAvgGradient(grad_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_AVERAGE:
    {
      grad_vec = getWeightedScaledAvgGradient(grad_set, n_dof_);
      break;
    }
    case GradientCombineMethod::LEAST_SQUARES:
    {
      grad_vec = getLeastSquaresGradient(grad_set, n_dof_, num_eq);
      break;
    }
    case GradientCombineMethod::WEIGHTED_LEAST_SQUARES:
    {
      grad_vec = getWeightedLeastSquaresGradient(grad_set, n_dof_, num_eq);
      // grad_vec = getWeightedLeastSquaresGradient2(grad_set, n_dof_, num_eq);
      break;
    }
  }

  // This does work but could be faster
  for (int j = 0; j < n_dof_; j++)
  {
    // Collision is 1 x n_dof
    jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
  }
}

void ContinuousCollisionConstraintIfopt::CalcJacobianBlockBothFree(Jacobian& jac_block,
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
  long num_eq{ 0 };
  GradientResultsSet grad_set(dist_results.size());
  for (tesseract_collision::ContactResult& dist_result : dist_results)
  {
    GradientResults result = collision_evaluator_->GetGradient(joint_vals0, joint_vals1, dist_result, isTimestep1);
    num_eq += (result.gradients[0].has_gradient + result.gradients[1].has_gradient);
    grad_set.add(result);
  }

  // Convert GradientResults to jacobian
  Eigen::VectorXd grad_vec;
  switch (gradient_method_)
  {
    case GradientCombineMethod::SUM:
    {
      grad_vec = getSumGradient(grad_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_SUM:
    {
      grad_vec = getSumGradient(grad_set, n_dof_);
      break;
    }
    case GradientCombineMethod::AVERAGE:
    {
      grad_vec = getAvgGradient(grad_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_AVERAGE:
    {
      grad_vec = getWeightedScaledAvgGradient(grad_set, n_dof_);
      break;
    }
    case GradientCombineMethod::LEAST_SQUARES:
    {
      grad_vec = getLeastSquaresGradient(grad_set, n_dof_, num_eq);
      break;
    }
    case GradientCombineMethod::WEIGHTED_LEAST_SQUARES:
    {
      grad_vec = getWeightedLeastSquaresGradient(grad_set, n_dof_, num_eq);
      // grad_vec = getWeightedLeastSquaresGradient2(grad_set, n_dof_, num_eq);
      break;
    }
  }

  // This does work but could be faster
  for (int j = 0; j < n_dof_; j++)
  {
    // Collision is 1 x n_dof
    jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
  }
}

ContinuousCollisionEvaluator::Ptr ContinuousCollisionConstraintIfopt::GetCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt
