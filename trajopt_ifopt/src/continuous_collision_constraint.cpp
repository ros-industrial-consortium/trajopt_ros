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
    std::array<JointPosition::ConstPtr, 3> position_vars,
    const std::string& name)
  : ifopt::ConstraintSet(1, name)
  , position_vars_(std::move(position_vars))
  , collision_evaluator_(std::move(collision_evaluator))
  , gradient_method_(gradient_method)
{
  if (position_vars_[1] == nullptr)
    throw std::runtime_error("position_vars index 1 should not be a nullptr!");

  if (position_vars_[0] == nullptr && position_vars_[2] == nullptr)
    throw std::runtime_error("position_vars index 0 and 2 are nullptr, atleast one should not be!");

  // Set n_dof_ for convenience
  n_dof_ = position_vars_[1]->GetRows();
  if (!(n_dof_ > 0))
    throw std::runtime_error("position_vars index 1 is empty!");

  if (position_vars_[0] != nullptr && position_vars_[0]->GetRows() != position_vars_[1]->GetRows())
    throw std::runtime_error("position_vars index 0 and 1 are not the same size!");

  if (position_vars_[2] != nullptr && position_vars_[2]->GetRows() != position_vars_[1]->GetRows())
    throw std::runtime_error("position_vars index 1 and 2 are not the same size!");

  if (position_vars_[0] != nullptr && position_vars_[2] != nullptr)
    mode_ = 0;  // central
  else if (position_vars_[2] != nullptr)
    mode_ = 1;  // forward
  else if (position_vars_[0] != nullptr)
    mode_ = 2;  // backward

  bounds_ = std::vector<ifopt::Bounds>(1, ifopt::BoundSmallerZero);
}

Eigen::VectorXd ContinuousCollisionConstraintIfopt::GetValues() const
{
  Eigen::VectorXd err = Eigen::VectorXd::Zero(1);
  // Get current joint values
  if (mode_ == 0)  // central, this will be the most common mode
  {
    Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_vars_[0]->GetName())->GetValues();
    Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
    Eigen::VectorXd joint_vals2 = this->GetVariables()->GetComponent(position_vars_[2]->GetName())->GetValues();
    err = CalcValuesCent(joint_vals0, joint_vals1, joint_vals2);
  }
  else if (mode_ == 1)  // Post
  {
    Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
    Eigen::VectorXd joint_vals2 = this->GetVariables()->GetComponent(position_vars_[2]->GetName())->GetValues();
    err = CalcValuesPost(joint_vals1, joint_vals2);
  }
  else if (mode_ == 2)  // Previous
  {
    Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_vars_[0]->GetName())->GetValues();
    Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
    err = CalcValuesPrev(joint_vals0, joint_vals1);
  }
  return err;
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> ContinuousCollisionConstraintIfopt::GetBounds() const { return bounds_; }

void ContinuousCollisionConstraintIfopt::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == position_vars_[1]->GetName())
  {
    if (mode_ == 0)  // central, this will be the most common mode
    {
      Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_vars_[0]->GetName())->GetValues();
      Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
      Eigen::VectorXd joint_vals2 = this->GetVariables()->GetComponent(position_vars_[2]->GetName())->GetValues();
      CalcJacobianBlockCent(jac_block, joint_vals0, joint_vals1, joint_vals2);
    }
    else if (mode_ == 1)  // Post
    {
      Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
      Eigen::VectorXd joint_vals2 = this->GetVariables()->GetComponent(position_vars_[2]->GetName())->GetValues();
      CalcJacobianBlockPost(jac_block, joint_vals1, joint_vals2);
    }
    else if (mode_ == 2)  // Previous
    {
      Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_vars_[0]->GetName())->GetValues();
      Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
      CalcJacobianBlockPrev(jac_block, joint_vals0, joint_vals1);
    }
  }

  //  if (var_set == position_var0_->GetName() &&
  //      collision_evaluator_->GetEvaluatorType() != ContinuousCollisionEvaluatorType::START_FIXED_END_FREE)
  //  {
  //    // Get current joint values
  //    VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_var0_->GetName())->GetValues();
  //    VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_var1_->GetName())->GetValues();

  //    if (collision_evaluator_->GetEvaluatorType() == ContinuousCollisionEvaluatorType::START_FREE_END_FREE)
  //      CalcJacobianBlockBothFree(jac_block, joint_vals0, joint_vals1);
  //    else
  //      CalcJacobianBlockStartFree(jac_block, joint_vals0, joint_vals1);
  //  }
  //  else if (var_set == position_var1_->GetName() &&
  //           collision_evaluator_->GetEvaluatorType() != ContinuousCollisionEvaluatorType::START_FREE_END_FIXED)
  //  {
  //    // Get current joint values
  //    VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_var0_->GetName())->GetValues();
  //    VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_var1_->GetName())->GetValues();

  //    if (collision_evaluator_->GetEvaluatorType() == ContinuousCollisionEvaluatorType::START_FREE_END_FREE)
  //      CalcJacobianBlockBothFree(jac_block, joint_vals0, joint_vals1);
  //    else
  //      CalcJacobianBlockEndFree(jac_block, joint_vals0, joint_vals1);
  //  }
}

Eigen::VectorXd
ContinuousCollisionConstraintIfopt::CalcValuesPost(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                                   const Eigen::Ref<const Eigen::VectorXd>& joint_vals_post) const
{
  Eigen::VectorXd err = Eigen::VectorXd::Zero(1);

  // Check the collisions
  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisionData(joint_vals, joint_vals_post);

  if (collision_data->contact_results_vector.empty() || !(collision_data->gradient_results_set.max_error > 0))
    return err;

  switch (gradient_method_)
  {
    case GradientCombineMethod::SUM:
    {
      for (const GradientResults& grad_result : collision_data->gradient_results_set.results)
        err[0] += std::max<double>(grad_result.error * grad_result.data[2], 0.);

      break;
    }
    case GradientCombineMethod::WEIGHTED_SUM:
    {
      for (const GradientResults& grad_result : collision_data->gradient_results_set.results)
      {
        double e = grad_result.error * grad_result.data[2];
        if (e > 0)
        {
          assert(collision_data->gradient_results_set.max_error_with_buffer > 0);
          double eb = std::max<double>(grad_result.error_with_buffer * grad_result.data[2], 0.);
          double we = eb / collision_data->gradient_results_set.max_error_with_buffer;
          err[0] += we * std::max<double>(e, 0.);
        }
      }

      break;
    }
    case GradientCombineMethod::AVERAGE:
    {
      long cnt{ 0 };
      for (const GradientResults& grad_result : collision_data->gradient_results_set.results)
      {
        double e = grad_result.error * grad_result.data[2];
        if (e > 0)
        {
          err[0] += std::max<double>(e, 0.);
          ++cnt;
        }
      }

      (cnt == 0) ? err[0] = 0 : err[0] = err[0] / double(cnt);

      break;
    }
    case GradientCombineMethod::WEIGHTED_AVERAGE:
    {
      double total_weight{ 0 };
      for (const GradientResults& grad_result : collision_data->gradient_results_set.results)
      {
        double d = std::max<double>(grad_result.error, 0.) * grad_result.data[2];
        double w = (std::max(grad_result.error_with_buffer * grad_result.data[2], 0.) /
                    collision_data->gradient_results_set.max_weighted_error_with_buffer);
        total_weight += w;
        err[0] += (w * d);
      }
      assert(total_weight > 0);
      err[0] = err[0] / total_weight;
      break;
    }
    case GradientCombineMethod::LEAST_SQUARES:
    {
      for (const GradientResults& grad_result : collision_data->gradient_results_set.results)
        err[0] += std::max<double>(grad_result.error * grad_result.data[2], 0.);

      err[0] = err[0] / static_cast<double>(collision_data->gradient_results_set.results.size());
      break;
    }
    case GradientCombineMethod::WEIGHTED_LEAST_SQUARES:
    {
      for (const GradientResults& grad_result : collision_data->gradient_results_set.results)
        err[0] += std::pow(std::max<double>(grad_result.error * grad_result.data[2], 0.), 2);

      err[0] = err[0] / static_cast<double>(collision_data->gradient_results_set.results.size());
      break;
    }
  }

  return err;
}

Eigen::VectorXd
ContinuousCollisionConstraintIfopt::CalcValuesPrev(const Eigen::Ref<const Eigen::VectorXd>& joint_vals_prev,
                                                   const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  Eigen::VectorXd err = Eigen::VectorXd::Zero(1);

  // Check the collisions
  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisionData(joint_vals_prev, joint_vals);

  if (collision_data->contact_results_vector.empty() || !(collision_data->gradient_results_set.max_error > 0))
    return err;

  switch (gradient_method_)
  {
    case GradientCombineMethod::SUM:
    {
      for (const GradientResults& grad_result : collision_data->gradient_results_set.results)
        err[0] += std::max<double>(grad_result.error * grad_result.data[2], 0.);

      break;
    }
    case GradientCombineMethod::WEIGHTED_SUM:
    {
      for (const GradientResults& grad_result : collision_data->gradient_results_set.results)
      {
        double e = grad_result.error * grad_result.data[2];
        if (e > 0)
        {
          assert(collision_data->gradient_results_set.max_error_with_buffer > 0);
          double eb = std::max<double>(grad_result.error_with_buffer * grad_result.data[2], 0.);
          double we = eb / collision_data->gradient_results_set.max_error_with_buffer;
          err[0] += we * std::max<double>(e, 0.);
        }
      }

      break;
    }
    case GradientCombineMethod::AVERAGE:
    {
      long cnt{ 0 };
      for (const GradientResults& grad_result : collision_data->gradient_results_set.results)
      {
        double e = grad_result.error * grad_result.data[2];
        if (e > 0)
        {
          err[0] += std::max<double>(e, 0.);
          ++cnt;
        }
      }

      (cnt == 0) ? err[0] = 0 : err[0] = err[0] / double(cnt);

      break;
    }
    case GradientCombineMethod::WEIGHTED_AVERAGE:
    {
      double total_weight{ 0 };
      for (const GradientResults& grad_result : collision_data->gradient_results_set.results)
      {
        double d = std::max<double>(grad_result.error, 0.) * grad_result.data[2];
        double w = (std::max(grad_result.error_with_buffer * grad_result.data[2], 0.) /
                    collision_data->gradient_results_set.max_weighted_error_with_buffer);
        total_weight += w;
        err[0] += (w * d);
      }
      assert(total_weight > 0);
      err[0] = err[0] / total_weight;
      break;
    }
    case GradientCombineMethod::LEAST_SQUARES:
    {
      for (const GradientResults& grad_result : collision_data->gradient_results_set.results)
        err[0] += std::max<double>(grad_result.error * grad_result.data[2], 0.);

      err[0] = err[0] / static_cast<double>(collision_data->gradient_results_set.results.size());
      break;
    }
    case GradientCombineMethod::WEIGHTED_LEAST_SQUARES:
    {
      for (const GradientResults& grad_result : collision_data->gradient_results_set.results)
        err[0] += std::pow(std::max<double>(grad_result.error * grad_result.data[2], 0.), 2);

      err[0] = err[0] / static_cast<double>(collision_data->gradient_results_set.results.size());
      break;
    }
  }

  return err;
}

Eigen::VectorXd
ContinuousCollisionConstraintIfopt::CalcValuesCent(const Eigen::Ref<const Eigen::VectorXd>& joint_vals_prev,
                                                   const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                                   const Eigen::Ref<const Eigen::VectorXd>& joint_vals_post) const
{
  Eigen::VectorXd err = Eigen::VectorXd::Zero(1);

  // Check the collisions
  std::array<CollisionCacheData::ConstPtr, 2> collision_data;
  collision_data[0] = collision_evaluator_->CalcCollisionData(joint_vals_prev, joint_vals);
  collision_data[1] = collision_evaluator_->CalcCollisionData(joint_vals, joint_vals_post);

  if (collision_data[0]->contact_results_vector.empty() && collision_data[1]->contact_results_vector.empty())
    return err;

  if (!(collision_data[0]->gradient_results_set.max_error > 0) &&
      !(collision_data[1]->gradient_results_set.max_error > 0))
    return err;

  switch (gradient_method_)
  {
    case GradientCombineMethod::SUM:
    {
      for (std::size_t i = 0; i < 2; ++i)
      {
        for (const GradientResults& grad_result : collision_data[i]->gradient_results_set.results)
          err[0] += std::max<double>(grad_result.error * grad_result.data[2], 0.);
      }

      break;
    }
    case GradientCombineMethod::WEIGHTED_SUM:
    {
      double max_error_with_buffer = std::max(collision_data[0]->gradient_results_set.max_error_with_buffer,
                                              collision_data[1]->gradient_results_set.max_error_with_buffer);
      for (std::size_t i = 0; i < 2; ++i)
      {
        for (const GradientResults& grad_result : collision_data[i]->gradient_results_set.results)
        {
          double e = grad_result.error * grad_result.data[2];
          if (e > 0)
          {
            assert(max_error_with_buffer > 0);
            double eb = std::max<double>(grad_result.error_with_buffer * grad_result.data[2], 0.);
            double we = eb / max_error_with_buffer;
            err[0] += we * std::max<double>(e, 0.);
          }
        }
      }

      break;
    }
    case GradientCombineMethod::AVERAGE:
    {
      long cnt{ 0 };
      for (std::size_t i = 0; i < 2; ++i)
      {
        for (const GradientResults& grad_result : collision_data[i]->gradient_results_set.results)
        {
          double e = grad_result.error * grad_result.data[2];
          if (e > 0)
          {
            err[0] += std::max<double>(e, 0.);
            ++cnt;
          }
        }
      }

      (cnt == 0) ? err[0] = 0 : err[0] = err[0] / double(cnt);

      break;
    }
    case GradientCombineMethod::WEIGHTED_AVERAGE:
    {
      double max_weighted_error_with_buffer =
          std::max(collision_data[0]->gradient_results_set.max_weighted_error_with_buffer,
                   collision_data[1]->gradient_results_set.max_weighted_error_with_buffer);
      double total_weight{ 0 };
      for (std::size_t i = 0; i < 2; ++i)
      {
        for (const GradientResults& grad_result : collision_data[i]->gradient_results_set.results)
        {
          double d = std::max<double>(grad_result.error, 0.) * grad_result.data[2];
          double w =
              (std::max(grad_result.error_with_buffer * grad_result.data[2], 0.) / max_weighted_error_with_buffer);
          total_weight += w;
          err[0] += (w * d);
        }
      }
      assert(total_weight > 0);
      err[0] = err[0] / total_weight;
      break;
    }
    case GradientCombineMethod::LEAST_SQUARES:
    {
      for (std::size_t i = 0; i < 2; ++i)
      {
        for (const GradientResults& grad_result : collision_data[i]->gradient_results_set.results)
          err[0] += std::max<double>(grad_result.error * grad_result.data[2], 0.);
      }

      err[0] = err[0] / static_cast<double>(collision_data[0]->gradient_results_set.results.size() +
                                            collision_data[1]->gradient_results_set.results.size());
      break;
    }
    case GradientCombineMethod::WEIGHTED_LEAST_SQUARES:
    {
      for (std::size_t i = 0; i < 2; ++i)
      {
        for (const GradientResults& grad_result : collision_data[i]->gradient_results_set.results)
          err[0] += std::pow(std::max<double>(grad_result.error * grad_result.data[2], 0.), 2);
      }

      err[0] = err[0] / static_cast<double>(collision_data[0]->gradient_results_set.results.size() +
                                            collision_data[1]->gradient_results_set.results.size());
      break;
    }
    default:
    {
      throw std::runtime_error("Invalid GradientCombineMethod.");
    }
  }

  return err;
}

void ContinuousCollisionConstraintIfopt::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 1);
  bounds_ = bounds;
}

void ContinuousCollisionConstraintIfopt::CalcJacobianBlockPost(
    Jacobian& jac_block,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals_post) const
{
  // Reserve enough room in the sparse matrix
  jac_block.reserve(n_dof_);

  // Calculate collisions
  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisionData(joint_vals, joint_vals_post);

  // Convert GradientResults to jacobian
  Eigen::VectorXd grad_vec;
  switch (gradient_method_)
  {
    case GradientCombineMethod::SUM:
    {
      grad_vec = getSumGradientPost(collision_data->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_SUM:
    {
      grad_vec = getWeightedSumGradientPost(collision_data->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::AVERAGE:
    {
      grad_vec = getAvgGradientPost(collision_data->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_AVERAGE:
    {
      grad_vec = getWeightedAvgGradientPost(collision_data->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::LEAST_SQUARES:
    {
      grad_vec = getLeastSquaresGradientPost(collision_data->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_LEAST_SQUARES:
    {
      grad_vec = getWeightedLeastSquaresGradientPost(collision_data->gradient_results_set, n_dof_);
      // grad_vec = getWeightedLeastSquaresGradient2(grad_set, n_dof_, num_eq);
      break;
    }
    default:
    {
      throw std::runtime_error("Invalid GradientCombineMethod.");
    }
  }

  // This does work but could be faster
  for (int j = 0; j < n_dof_; j++)
  {
    // Collision is 1 x n_dof
    jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
  }
}

void ContinuousCollisionConstraintIfopt::CalcJacobianBlockCent(
    Jacobian& jac_block,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals_prev,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals_post) const
{
  // Reserve enough room in the sparse matrix
  jac_block.reserve(n_dof_);

  // Calculate collisions
  CollisionCacheData::ConstPtr collision_data_prev =
      collision_evaluator_->CalcCollisionData(joint_vals_prev, joint_vals);
  CollisionCacheData::ConstPtr collision_data_post =
      collision_evaluator_->CalcCollisionData(joint_vals, joint_vals_post);

  // Convert GradientResults to jacobian
  Eigen::VectorXd grad_vec;
  switch (gradient_method_)
  {
    case GradientCombineMethod::SUM:
    {
      grad_vec = getSumGradientCent(
          collision_data_prev->gradient_results_set, collision_data_post->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_SUM:
    {
      grad_vec = getWeightedSumGradientCent(
          collision_data_prev->gradient_results_set, collision_data_post->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::AVERAGE:
    {
      grad_vec = getAvgGradientCent(
          collision_data_prev->gradient_results_set, collision_data_post->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_AVERAGE:
    {
      grad_vec = getWeightedAvgGradientCent(
          collision_data_prev->gradient_results_set, collision_data_post->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::LEAST_SQUARES:
    {
      grad_vec = getLeastSquaresGradientCent(
          collision_data_prev->gradient_results_set, collision_data_post->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_LEAST_SQUARES:
    {
      grad_vec = getWeightedLeastSquaresGradientCent(
          collision_data_prev->gradient_results_set, collision_data_post->gradient_results_set, n_dof_);
      // grad_vec = getWeightedLeastSquaresGradient2(grad_set, n_dof_, num_eq);
      break;
    }
    default:
    {
      throw std::runtime_error("Invalid GradientCombineMethod.");
    }
  }

  // This does work but could be faster
  for (int j = 0; j < n_dof_; j++)
  {
    // Collision is 1 x n_dof
    jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
  }
}

void ContinuousCollisionConstraintIfopt::CalcJacobianBlockPrev(
    Jacobian& jac_block,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals_pre,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  // Reserve enough room in the sparse matrix
  jac_block.reserve(n_dof_);

  // Calculate collisions
  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisionData(joint_vals_pre, joint_vals);

  // Convert GradientResults to jacobian
  Eigen::VectorXd grad_vec;
  switch (gradient_method_)
  {
    case GradientCombineMethod::SUM:
    {
      grad_vec = getSumGradientPrev(collision_data->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_SUM:
    {
      grad_vec = getWeightedSumGradientPrev(collision_data->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::AVERAGE:
    {
      grad_vec = getAvgGradientPrev(collision_data->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_AVERAGE:
    {
      grad_vec = getWeightedAvgGradientPrev(collision_data->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::LEAST_SQUARES:
    {
      grad_vec = getLeastSquaresGradientPrev(collision_data->gradient_results_set, n_dof_);
      break;
    }
    case GradientCombineMethod::WEIGHTED_LEAST_SQUARES:
    {
      grad_vec = getWeightedLeastSquaresGradientPrev(collision_data->gradient_results_set, n_dof_);
      // grad_vec = getWeightedLeastSquaresGradient2(grad_set, n_dof_, num_eq);
      break;
    }
    default:
    {
      throw std::runtime_error("Invalid GradientCombineMethod.");
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
