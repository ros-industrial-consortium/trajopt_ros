/**
 * @file joint_velocity_variable.h
 * @brief Contains the joint velocity variable
 *
 * @author Ben Greenberg
 * @date April 22, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TRAJOPT_IFOPT_JOINT_VELOCITY_VARIABLE_H
#define TRAJOPT_IFOPT_JOINT_VELOCITY_VARIABLE_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/variable_set.h>
#include <ifopt/bounds.h>
#include <Eigen/Eigen>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/utils/ifopt_utils.h>

namespace trajopt
{
/**
 * @brief Represents a single joint velocity in the optimization. Values are of dimension 1 x n_dof
 */
class JointVelocity : public ifopt::VariableSet
{
public:
  using Ptr = std::shared_ptr<JointVelocity>;
  using ConstPtr = std::shared_ptr<const JointVelocity>;

  JointVelocity(const Eigen::Ref<const Eigen::VectorXd>& init_value,
                std::vector<std::string> joint_names,
                const std::string& name = "Joint_Velocity")
    : ifopt::VariableSet(static_cast<int>(init_value.size()), name), joint_names_(std::move(joint_names))
  {
    // This needs to be set somehow
    ifopt::Bounds bounds(-M_PI, M_PI);
    bounds_ = std::vector<ifopt::Bounds>(static_cast<size_t>(init_value.size()), bounds);
    values_ = init_value;
  }

  /**
   * @brief Sets this variable to the given joint velocity
   * @param x Joint Position to which this variable will be set.
   */
  void SetVariables(const Eigen::VectorXd& x) override { values_ = x; }

  /**
   * @brief Gets the joint velocity associated with this variable
   * @return This variable's joint velocity
   */
  Eigen::VectorXd GetValues() const override { return values_; }

  /**
   * @brief Gets the bounds on this variable
   * @return Bounds on this variable
   */
  VecBound GetBounds() const override { return bounds_; }

  /**
   * @brief Sets the bounds for the joints in this variable.
   * @param new_bounds New bounds for the joints
   */
  void SetBounds(VecBound& new_bounds) { bounds_ = new_bounds; }

  /**
   * @brief Sets the bounds for the joints in this variable from a MatrixX2d with the first column being lower bound and
   * second column being upper bound
   * @param bounds Columns 1/2 are lower/upper bounds. You probably will get this from forward_kinematics->getLimits()
   */
  void SetBounds(const Eigen::Ref<const Eigen::MatrixX2d>& bounds) { bounds_ = toBounds(bounds); }

  /**
   * @brief Get the joint names associated with this variable set
   * @return The joint names
   */
  std::vector<std::string> GetJointNames() const { return joint_names_; }

private:
  VecBound bounds_;
  Eigen::VectorXd values_;
  std::vector<std::string> joint_names_;
};

}  // namespace trajopt

#endif
