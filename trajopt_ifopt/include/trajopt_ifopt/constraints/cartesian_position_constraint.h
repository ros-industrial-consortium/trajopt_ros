/**
 * @file cartesian_position_constraint.h
 * @brief The cartesian position constraint
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
#ifndef TRAJOPT_IFOPT_CARTESIAN_POSITION_CONSTRAINT_H
#define TRAJOPT_IFOPT_CARTESIAN_POSITION_CONSTRAINT_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <ifopt/constraint_set.h>

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/utils.hpp>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/kinematics_info.h>

namespace trajopt_ifopt
{
/** @brief Contains Cartesian pose constraint information */
struct CartPosInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CartPosInfo>;
  using ConstPtr = std::shared_ptr<const CartPosInfo>;

  CartPosInfo() = default;
  CartPosInfo(KinematicsInfo::ConstPtr kin_info,
              const Eigen::Isometry3d& target_pose,
              std::string source_link,
              const Eigen::Isometry3d& source_tcp = Eigen::Isometry3d::Identity(),
              Eigen::VectorXi indices = Eigen::Matrix<int, 1, 6>(std::vector<int>({ 0, 1, 2, 3, 4, 5 }).data()))
    : kin_info(std::move(kin_info))
    , source_link(std::move(source_link))
    , source_tcp(source_tcp)
    , target_pose(target_pose)
    , target_pose_inv(target_pose.inverse())
    , indices(std::move(indices))
  {
    this->source_kin_link = this->kin_info->adjacency_map->getLinkMapping(this->source_link);
    if (this->source_kin_link == nullptr)
      throw std::runtime_error("CartPosKinematicInfo: Source Link name '" + this->source_link +
                               "' provided does not exist.");

    if (this->indices.size() > 6)
      throw std::runtime_error("CartPosKinematicInfo: The indicies list length cannot be larger than six.");

    if (this->indices.size() == 0)
      throw std::runtime_error("CartPosKinematicInfo: The indicies list length is zero.");
  }

  /** @brief The kinematics information */
  KinematicsInfo::ConstPtr kin_info;

  /** @brief Link which should reach desired pos */
  std::string source_link;

  /** @brief Static transform applied to the link_ location */
  Eigen::Isometry3d source_tcp;

  /** @brief This is a map containing the transform from link_ to its adjacent moving link in the kinematics object */
  tesseract_environment::AdjacencyMapPair::ConstPtr source_kin_link;

  /** @brief The target TCP pose in world frame. Not used for calculation. Stored for convenience */
  Eigen::Isometry3d target_pose;

  /** @brief The inverse of target_pose_ used for error calculations */
  Eigen::Isometry3d target_pose_inv;

  /**
   * @brief This is a vector of indices to be returned Default: {0, 1, 2, 3, 4, 5}
   *
   * If you only care about x, y and z error, this is {0, 1, 2}
   * If you only care about rotation error around x, y and z, this is {3, 4, 5}
   */
  Eigen::VectorXi indices;
};

class CartPosConstraint : public ifopt::ConstraintSet
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CartPosConstraint>;
  using ConstPtr = std::shared_ptr<const CartPosConstraint>;

  CartPosConstraint(CartPosInfo info, JointPosition::ConstPtr position_var, const std::string& name = "CartPos");

  CartPosConstraint(CartPosInfo info,
                    JointPosition::ConstPtr position_var,
                    const Eigen::VectorXd& coeffs,
                    const std::string& name = "CartPos");

  /**
   * @brief CalcValues Calculates the values associated with the constraint
   * @param joint_vals Input joint values for which FK is solved
   * @return Error of FK solution from target, size 6. The first 3 terms are associated with position and the last 3 are
   * associated with orientation.
   */
  Eigen::VectorXd CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const;
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

  void SetBounds(const std::vector<ifopt::Bounds>& bounds);

  /**
   * @brief Fills the jacobian block associated with the constraint
   * @param jac_block Block of the overall jacobian associated with these constraints
   */
  void CalcJacobianBlock(const Eigen::Ref<const Eigen::VectorXd>& joint_vals, Jacobian& jac_block) const;
  /**
   * @brief Fills the jacobian block associated with the given var_set.
   * @param var_set Name of the var_set to which the jac_block is associated
   * @param jac_block Block of the overal jacobian associated with these constraints and the var_set variable
   */
  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

  /**
   * @brief Gets the Cartesian Pose info used to create this constraint
   * @return The Cartesian Pose info used to create this constraint
   */
  const CartPosInfo& GetInfo() const { return info_; }
  CartPosInfo& GetInfo() { return info_; }

  /**
   * @brief Set the target pose
   * @param pose
   */
  void SetTargetPose(const Eigen::Isometry3d& pose)
  {
    info_.target_pose = pose;
    info_.target_pose_inv = pose.inverse();
  }

  /**
   * @brief Returns the target pose for the constraint
   * @return The target pose for the constraint
   */
  Eigen::Isometry3d GetTargetPose() const { return info_.target_pose; }

  /**
   * @brief Returns the current TCP pose in world frame given the input kinematic info and the current variable values
   * @return The current TCP pose given the input kinematic info and the current variable values
   */
  Eigen::Isometry3d GetCurrentPose() const;

  /** @brief If true, numeric differentiation will be used. Default: true
   *
   * Note: While the logic for using the jacobian from KDL will be used if set to false, this has been buggy. Set this
   * to false at your own risk.
   */
  bool use_numeric_differentiation{ true };

private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_;

  /** @brief The constraint coefficients */
  Eigen::VectorXd coeffs_;

  /** @brief Bounds on the positions of each joint */
  std::vector<ifopt::Bounds> bounds_;

  /**
   * @brief Pointers to the vars used by this constraint.
   * Do not access them directly. Instead use this->GetVariables()->GetComponent(position_var->GetName())->GetValues()
   */
  JointPosition::ConstPtr position_var_;

  /** @brief The kinematic information used when calculating error */
  CartPosInfo info_;
};
};  // namespace trajopt_ifopt
#endif
