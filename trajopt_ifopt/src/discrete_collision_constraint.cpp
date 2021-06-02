/**
 * @file discrete_collision_constraint.cpp
 * @brief The single timestep collision position constraint
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
#include <trajopt_ifopt/constraints/discrete_collision_constraint.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_kinematics/core/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision_utils.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>

namespace trajopt
{
DiscreteCollisionConstraintIfopt::DiscreteCollisionConstraintIfopt(DiscreteCollisionEvaluator::Ptr collision_evaluator,
                                                                   GradientCombineMethod gradient_method,
                                                                   JointPosition::ConstPtr position_var,
                                                                   const std::string& name)
  : ifopt::ConstraintSet(1, name)
  , position_var_(std::move(position_var))
  , collision_evaluator_(std::move(collision_evaluator))
  , gradient_method_(gradient_method)
{
  // Set n_dof_ for convenience
  n_dof_ = position_var_->GetRows();
  assert(n_dof_ > 0);

  bounds_ = std::vector<ifopt::Bounds>(1, ifopt::BoundSmallerZero);
}

Eigen::VectorXd DiscreteCollisionConstraintIfopt::GetValues() const
{
  // Get current joint values
  Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();

  return CalcValues(joint_vals);
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> DiscreteCollisionConstraintIfopt::GetBounds() const { return bounds_; }

void DiscreteCollisionConstraintIfopt::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == position_var_->GetName())
  {
    // Get current joint values
    VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();

    CalcJacobianBlock(joint_vals, jac_block);
  }
}

Eigen::VectorXd DiscreteCollisionConstraintIfopt::CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  Eigen::VectorXd err = Eigen::VectorXd::Zero(1);

  // Check the collisions
  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisions(joint_vals);

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
          //          assert(collision_data->gradient_results_set.max_weighted_error_with_buffer > 0);
          //          double eb = std::max<double>(grad_result.error_with_buffer * grad_result.data[2], 0.);
          //          double we = eb / collision_data->gradient_results_set.max_weighted_error_with_buffer;
          err[0] += std::max<double>(e, 0.);
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

void DiscreteCollisionConstraintIfopt::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 1);
  bounds_ = bounds;
}

void DiscreteCollisionConstraintIfopt::CalcJacobianBlock(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                                         Jacobian& jac_block) const
{
  // Reserve enough room in the sparse matrix
  jac_block.reserve(n_dof_);

  // Calculate collisions
  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisions(joint_vals);

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
  }

  // This does work but could be faster
  for (int j = 0; j < n_dof_; j++)
  {
    // Collision is 1 x n_dof
    jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
  }
}

DiscreteCollisionEvaluator::Ptr DiscreteCollisionConstraintIfopt::GetCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt
