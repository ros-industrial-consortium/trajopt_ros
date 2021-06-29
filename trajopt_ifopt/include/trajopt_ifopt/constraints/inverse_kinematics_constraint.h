/**
 * @file inverse_kinematics_constraint.h
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
#ifndef TRAJOPT_IFOPT_INVERSE_KINEMATICS_CONSTRAINT_H
#define TRAJOPT_IFOPT_INVERSE_KINEMATICS_CONSTRAINT_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <ifopt/constraint_set.h>

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/utils.hpp>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

namespace trajopt_ifopt
{
/**
 * @brief Contains kinematic information for the inverse kinematics constraint
 */
struct InverseKinematicsInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<InverseKinematicsInfo>;
  using ConstPtr = std::shared_ptr<const InverseKinematicsInfo>;

  InverseKinematicsInfo() = default;
  InverseKinematicsInfo(tesseract_kinematics::InverseKinematics::ConstPtr inverse_kinematics,
                        tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                        const Eigen::Isometry3d& world_to_base,
                        std::string link,
                        const Eigen::Isometry3d& tcp = Eigen::Isometry3d::Identity())
    : inverse_kinematics(std::move(inverse_kinematics))
    , adjacency_map(std::move(adjacency_map))
    , world_to_base(world_to_base)
    , link(std::move(link))
    , tcp(tcp)
  {
    this->kin_link = this->adjacency_map->getLinkMapping(this->link);
    if (this->kin_link == nullptr)
    {
      CONSOLE_BRIDGE_logError("Link name '%s' provided does not exist.", this->link.c_str());
      assert(false);
    }
  }

  tesseract_kinematics::InverseKinematics::ConstPtr inverse_kinematics;
  /** @brief Not currently respected */
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map;
  /** @brief Not currently respected */
  Eigen::Isometry3d world_to_base;
  /** @brief Not currently respected */
  std::string link;
  /** @brief Not currently respected */
  tesseract_environment::AdjacencyMapPair::ConstPtr kin_link;
  /** @brief Not currently respected */
  Eigen::Isometry3d tcp;
};

/**
 * @brief This is a constraint on the distance of a joint position variable from an IK solution.
 *
 * IK is solved using the seed and the error is the distance from that solution. Bounds can then be set on the allowed
 * deviation for each joint
 *
 * TODO: Look into integration with descartes_light samplers to allow z-free
 */
class InverseKinematicsConstraint : public ifopt::ConstraintSet
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<InverseKinematicsConstraint>;
  using ConstPtr = std::shared_ptr<const InverseKinematicsConstraint>;

  InverseKinematicsConstraint(const Eigen::Isometry3d& target_pose,
                              InverseKinematicsInfo::ConstPtr kinematic_info,
                              JointPosition::ConstPtr constraint_var,
                              JointPosition::ConstPtr seed_var,
                              const std::string& name = "InverseKinematics");

  /**
   * @brief Calculates the values associated with the constraint
   * @param joint_vals Joint values for which the value is calculated
   * @param seed_joint_position Joint values used as the seed when calculating IK
   * @return Distance of each joint from the IK solution
   */
  Eigen::VectorXd CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                             const Eigen::Ref<const Eigen::VectorXd>& seed_joint_position) const;
  /**
   * @brief Returns the values associated with the constraint. This is the joint distance from the target joint position
   * (size n_dof_)
   * @return
   */
  Eigen::VectorXd GetValues() const override;

  /**
   * @brief  Returns the "bounds" of this constraint. How these are enforced is up to the solver
   * @return Bounds on the distance a joint can vary from the IK solution
   */
  std::vector<ifopt::Bounds> GetBounds() const override;

  /** @brief Set the constraint bounds. Must be n_dof_ */
  void SetBounds(const std::vector<ifopt::Bounds>& bounds);

  /**
   * @brief Fills the jacobian block associated with the constraint
   * @param jac_block Block of the overall jacobian associated with these constraints
   */
  void CalcJacobianBlock(const Eigen::Ref<const Eigen::VectorXd>& joint_vals, Jacobian& jac_block) const;
  /**
   * @brief Fills the jacobian block associated with the given var_set.
   *
   * Since the value of this constraint is the joint distance from the joint position acquired with IK, the jacobian is
   * the same as that for the joint position constraint.
   * @param var_set Name of the var_set to which the jac_block is associated
   * @param jac_block Block of the overal jacobian associated with these constraints and the var_set variable
   */
  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

  /**
   * @brief Sets the target pose for the TCP
   * @param target_pose Target pose for the TCP. Currently in robot frame since world_to_base_ has not been implemented
   */
  void SetTargetPose(const Eigen::Isometry3d& target_pose);

  /**
   * @brief Gets the kinematic info used to create this constraint
   * @return The kinematic info used to create this constraint
   */
  const InverseKinematicsInfo::ConstPtr& getKinematicInfo() { return kinematic_info_; }

private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_;

  /** @brief Bounds on the joint distance the constraint_var may vary from the IK solution */
  std::vector<ifopt::Bounds> bounds_;

  /** @brief Pointer to the var used by this constraint.
   *
   * Do not access them directly. Instead use this->GetVariables()->GetComponent(position_var->GetName())->GetValues()*/
  JointPosition::ConstPtr constraint_var_;
  /** @brief Pointer to the var used as a seed when calculating IK. This will usually be a adjacent point in the
   * trajectory*/
  JointPosition::ConstPtr seed_var_;
  /** @brief Target pose for the TCP. Currently in robot frame since world_to_base_ has not been implemented */
  Eigen::Isometry3d target_pose_;
  /** @brief The kinematic info used to create this constraint */
  InverseKinematicsInfo::ConstPtr kinematic_info_;
};
};  // namespace trajopt_ifopt
#endif
