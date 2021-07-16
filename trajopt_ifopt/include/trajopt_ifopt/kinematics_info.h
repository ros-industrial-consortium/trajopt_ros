/**
 * @file kinematics_info.h
 * @brief Kinematics information used throughout trajopt_ifopt
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
#ifndef TRAJOPT_IFOPT_KINEMATICS_INFO_H
#define TRAJOPT_IFOPT_KINEMATICS_INFO_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
/** @brief Contains kinematics information */
struct KinematicsInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<KinematicsInfo>;
  using ConstPtr = std::shared_ptr<const KinematicsInfo>;

  KinematicsInfo(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                 tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                 const Eigen::Isometry3d& world_to_base)
    : manip(std::move(manip)), adjacency_map(std::move(adjacency_map)), world_to_base(world_to_base)
  {
  }

  /** @brief Manipulator kinematics object */
  tesseract_kinematics::ForwardKinematics::ConstPtr manip;

  /** @brief Adjacency map for kinematics object mapping rigid links to moving links */
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map;

  /** @brief Transform from world (root of scene) to base of the kinematics object */
  Eigen::Isometry3d world_to_base;
};
}  // namespace trajopt_ifopt

#endif  // TRAJOPT_IFOPT_KINEMATICS_INFO_H
