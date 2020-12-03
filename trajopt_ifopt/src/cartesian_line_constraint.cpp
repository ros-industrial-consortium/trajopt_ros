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
#include <trajopt_ifopt/constraints/cartesian_line_constraint.h>
#include <trajopt_ifopt/utils/numeric_differentiation.h>

TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_kinematics/core/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt
{
CartLineConstraint::CartLineConstraint(const Eigen::Isometry3d& origin_pose,
                                      const Eigen::Isometry3d& target_pose,
                                      CartPosKinematicInfo::ConstPtr kinematic_info,
                                      JointPosition::ConstPtr position_var,
                                      const std::string& name)
  : ifopt::ConstraintSet(6, name)
  , position_var_(std::move(position_var))
  , line_start_(origin_pose)
  , line_end_(target_pose)
  , kinematic_info_(std::move(kinematic_info))
{
  // Set the n_dof and n_vars for convenience
  n_dof_ = kinematic_info_->manip->numJoints();
  assert(n_dof_ > 0);

  bounds_ = std::vector<ifopt::Bounds>(6, ifopt::BoundZero);

  // calculate the equation of the constraint line
  line_  = line_end_.translation() - line_start_.translation();//<< line_end_.translation().x() - line_start_.translation().x(), line_end_.translation().y() - line_start_.translation().y(), line_end_.translation().z() - line_start_.translation().z();

}

Eigen::VectorXd CartLineConstraint::CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals)
{

  Eigen::Isometry3d new_pose;
  kinematic_info_->manip->calcFwdKin(new_pose, joint_vals, kinematic_info_->kin_link->link_name);

  new_pose = kinematic_info_->world_to_base * new_pose * kinematic_info_->kin_link->transform * kinematic_info_->tcp;

  // For Jacobian Calc, we need the inverse of the nearest point, D, to new Pose, C, on the constraint line AB
  // This will not honor tcp orientation. Not used bc this returns a scalar error, need vector/direction components
//  Eigen::Vector3d d1,d2, pose_err;

  //AC, new pose to first point on line
//  d1 = new_pose.translation() - line_start_.translation();
  //BC, new pose to second point on line
//  d2 = new_pose.translation() - line_end_.translation();
  //Distance = |AC * BC| / |AB|

//  pose_err = d1.cross(d2).squaredNorm() /  line_.squaredNorm();

  //Find D, the closest point on the line to C, the new point

  //AC, new pose to first point on line
  Eigen::Vector3d d1 = new_pose.translation() - line_start_.translation();

  // ((AB * AC ) / (|AB|^2)) * (AB)
  Eigen::Isometry3d temp_ = Eigen::Isometry3d::Identity();
  double line_point_dist_ = (line_.dot(d1) / line_.dot(line_));
  if (line_point_dist_ > 1.0)
  {
      temp_.translate(line_end_.translation());
  }
  else if (line_point_dist_ < 0)
  {
      temp_.translate(line_start_.translation());
  }
  else
  {
      temp_.translate(line_point_dist_ * line_);
  }
  line_point_ = temp_;
  //SetLinePoint(temp_);

  //pose error is the vector from new pose C to nearest point on line AB, D
  Eigen::Vector3d pose_err = line_point_.translation() - new_pose.translation();

  // @TODO: Handle orientation
  Eigen::Vector4d rot_ = Eigen::Vector4d::Identity();
  Eigen::VectorXd err = concat(pose_err, rot_); //calcRotationalError(pose_err.rotation()));

  return err;
}

// @todo: Does this need to be updated
Eigen::VectorXd CartLineConstraint::GetValues()
{
  Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
  return CalcValues(joint_vals);
}

//void SetLinePoint(const Eigen::Isometry3d line_point)
//{
//  line_point_ = line_point;
//  line_point_inverse_ = line_point_.inverse();
//}


// Set the limits on the constraint values
std::vector<ifopt::Bounds> CartLineConstraint::GetBounds() const { return bounds_; }

void CartLineConstraint::SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 6);
  bounds_ = bounds;
}

void CartLineConstraint::CalcJacobianBlock(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                          Jacobian& jac_block)
{
  if (use_numeric_differentiation)
  {
    auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) { return this->CalcValues(x); };
    Jacobian jac0(6, n_dof_);
    jac0 = calcForwardNumJac(error_calculator, joint_vals, 1e-5);

    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        jac_block.coeffRef(i, j) = jac0.coeffRef(i, j);
      }
    }
  }
  else
  {
    // Reserve enough room in the sparse matrix
    jac_block.reserve(n_dof_ * 6);

    Eigen::MatrixXd jac0(6, n_dof_);
    Eigen::Isometry3d tf0;

    // Calculate the jacobian
    // @todo: I do not think this is the correct inverse
    kinematic_info_->manip->calcFwdKin(tf0, joint_vals, kinematic_info_->kin_link->link_name);
    kinematic_info_->manip->calcJacobian(jac0, joint_vals, kinematic_info_->kin_link->link_name);
    tesseract_kinematics::jacobianChangeBase(jac0, kinematic_info_->world_to_base);
    tesseract_kinematics::jacobianChangeRefPoint(
        jac0,
        (kinematic_info_->world_to_base * tf0).linear() *
            (kinematic_info_->kin_link->transform * kinematic_info_->tcp).translation());
    tesseract_kinematics::jacobianChangeBase(jac0, line_point_inv_);

    // Paper:
    // https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
    // The jacobian of the robot is the geometric jacobian (Je) which maps generalized velocities in
    // joint space to time derivatives of the end-effector configuration representation. It does not
    // represent the analytic jacobian (Ja) given by a partial differentiation of position and rotation
    // to generalized coordinates. Since the geometric jacobian is unique there exists a linear mapping
    // between velocities and the derivatives of the representation.
    //
    // The approach in the paper was tried but it was having issues with getting correct jacobian.
    // Must of had an error in the implementation so should revisit at another time but the approach
    // below should be sufficient and faster than numerical calculations using the err function.

    // The approach below leverages the geometric jacobian and a small step in time to approximate
    // the partial derivative of the error function. Note that the rotational portion is the only part
    // that is required to be modified per the paper.
    Eigen::Isometry3d pose_err = line_point_inv_ * tf0;
    Eigen::Vector3d rot_err = calcRotationalError(pose_err.rotation());
    for (int c = 0; c < jac0.cols(); ++c)
    {
      auto new_pose_err = addTwist(pose_err, jac0.col(c), 1e-5);
      Eigen::VectorXd new_rot_err = calcRotationalError(new_pose_err.rotation());
      jac0.col(c).tail(3) = ((new_rot_err - rot_err) / 1e-5);
    }

    // Convert to a sparse matrix and set the jacobian
    // TODO: Make this more efficient. This does not work.
    //    Jacobian jac_block = jac0.sparseView();

    // This does work but could be faster
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < n_dof_; j++)
      {
        // Each jac_block will be for a single variable but for all timesteps. Therefore we must index down to the
        // correct timestep for this variable
        jac_block.coeffRef(i, j) = jac0(i, j);
      }
    }
  }
}

void CartLineConstraint::FillJacobianBlock(std::string var_set, Jacobian& jac_block)
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == position_var_->GetName())
  {
    // Get current joint values and calculate jacobian
    Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
    CalcJacobianBlock(joint_vals, jac_block);
  }
}

void CartLineConstraint::SetLine(const Eigen::Isometry3d& Point_A, const Eigen::Isometry3d& Point_B)
{
  line_end_ = Point_B;
  line_start_ = Point_A;
  }

Eigen::Isometry3d CartLineConstraint::GetCurrentPose()
{
  Eigen:VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();
  Eigen::Isometry3d new_pose;
  kinematic_info_->manip->calcFwdKin(new_pose, joint_vals, kinematic_info_->kin_link->link_name);
  new_pose = kinematic_info_->world_to_base * new_pose * kinematic_info_->kin_link->transform * kinematic_info_->tcp;
  return new_pose;
}
}  // namespace trajopt
