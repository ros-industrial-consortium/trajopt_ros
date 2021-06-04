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

namespace trajopt
{
ContinuousCollisionConstraintIfopt::ContinuousCollisionConstraintIfopt(
    ContinuousCollisionEvaluator::Ptr collision_evaluator,
    ContinuousCombineCollisionData combine_methods,
    std::array<JointPosition::ConstPtr, 3> position_vars,
    const std::string& name)
  : ifopt::ConstraintSet(1, name)
  , position_vars_(std::move(position_vars))
  , collision_evaluator_(std::move(collision_evaluator))
  , combine_methods_(combine_methods)
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
    mode_ = GradientMode::CENT;  // central
  else if (position_vars_[2] != nullptr)
    mode_ = GradientMode::POST;  // post
  else if (position_vars_[0] != nullptr)
    mode_ = GradientMode::PREV;  // previous

  bounds_ = std::vector<ifopt::Bounds>(1, ifopt::BoundSmallerZero);
}

Eigen::VectorXd ContinuousCollisionConstraintIfopt::GetValues() const
{
  Eigen::VectorXd err = Eigen::VectorXd::Zero(1);
  // Get current joint values
  switch (mode_)
  {
    case GradientMode::CENT:  // this will be the most common mode
    {
      Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_vars_[0]->GetName())->GetValues();
      Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
      Eigen::VectorXd joint_vals2 = this->GetVariables()->GetComponent(position_vars_[2]->GetName())->GetValues();
      err = CalcValuesCent(joint_vals0, joint_vals1, joint_vals2);
      break;
    }
    case GradientMode::POST:
    {
      Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
      Eigen::VectorXd joint_vals2 = this->GetVariables()->GetComponent(position_vars_[2]->GetName())->GetValues();
      err = CalcValuesPost(joint_vals1, joint_vals2);
      break;
    }
    case GradientMode::PREV:
    {
      Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_vars_[0]->GetName())->GetValues();
      Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
      err = CalcValuesPrev(joint_vals0, joint_vals1);
      break;
    }
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
    switch (mode_)
    {
      case GradientMode::CENT:  // this will be the most common mode
      {
        Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_vars_[0]->GetName())->GetValues();
        Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
        Eigen::VectorXd joint_vals2 = this->GetVariables()->GetComponent(position_vars_[2]->GetName())->GetValues();
        CalcJacobianBlockCent(jac_block, joint_vals0, joint_vals1, joint_vals2);
        break;
      }
      case GradientMode::POST:
      {
        Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
        Eigen::VectorXd joint_vals2 = this->GetVariables()->GetComponent(position_vars_[2]->GetName())->GetValues();
        CalcJacobianBlockPost(jac_block, joint_vals1, joint_vals2);
        break;
      }
      case GradientMode::PREV:
      {
        Eigen::VectorXd joint_vals0 = this->GetVariables()->GetComponent(position_vars_[0]->GetName())->GetValues();
        Eigen::VectorXd joint_vals1 = this->GetVariables()->GetComponent(position_vars_[1]->GetName())->GetValues();
        CalcJacobianBlockPrev(jac_block, joint_vals0, joint_vals1);
        break;
      }
    }
  }
}

Eigen::VectorXd
ContinuousCollisionConstraintIfopt::CalcValuesPost(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                                   const Eigen::Ref<const Eigen::VectorXd>& joint_vals_post) const
{
  // Check the collisions
  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisionData(joint_vals, joint_vals_post);
  return combine_methods_.combine_values_post(*collision_data);
}

Eigen::VectorXd
ContinuousCollisionConstraintIfopt::CalcValuesPrev(const Eigen::Ref<const Eigen::VectorXd>& joint_vals_prev,
                                                   const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  // Check the collisions
  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisionData(joint_vals_prev, joint_vals);
  return combine_methods_.combine_values_prev(*collision_data);
}

Eigen::VectorXd
ContinuousCollisionConstraintIfopt::CalcValuesCent(const Eigen::Ref<const Eigen::VectorXd>& joint_vals_prev,
                                                   const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                                   const Eigen::Ref<const Eigen::VectorXd>& joint_vals_post) const
{
  // Check the collisions
  CollisionCacheData::ConstPtr collision_data0 = collision_evaluator_->CalcCollisionData(joint_vals_prev, joint_vals);
  CollisionCacheData::ConstPtr collision_data1 = collision_evaluator_->CalcCollisionData(joint_vals, joint_vals_post);
  return combine_methods_.combine_values_cent(*collision_data0, *collision_data1);
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
  // Calculate collisions
  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisionData(joint_vals, joint_vals_post);
  return combine_methods_.combine_jacobian_post(jac_block, *collision_data);
}

void ContinuousCollisionConstraintIfopt::CalcJacobianBlockCent(
    Jacobian& jac_block,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals_prev,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals_post) const
{
  // Calculate collisions
  CollisionCacheData::ConstPtr collision_data_prev =
      collision_evaluator_->CalcCollisionData(joint_vals_prev, joint_vals);
  CollisionCacheData::ConstPtr collision_data_post =
      collision_evaluator_->CalcCollisionData(joint_vals, joint_vals_post);
  return combine_methods_.combine_jacobian_cent(jac_block, *collision_data_prev, *collision_data_post);
}

void ContinuousCollisionConstraintIfopt::CalcJacobianBlockPrev(
    Jacobian& jac_block,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals_pre,
    const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  // Calculate collisions
  CollisionCacheData::ConstPtr collision_data = collision_evaluator_->CalcCollisionData(joint_vals_pre, joint_vals);
  return combine_methods_.combine_jacobian_prev(jac_block, *collision_data);
}

ContinuousCollisionEvaluator::Ptr ContinuousCollisionConstraintIfopt::GetCollisionEvaluator() const
{
  return collision_evaluator_;
}

}  // namespace trajopt
