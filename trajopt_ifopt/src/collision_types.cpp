/**
 * @file collision_types.h
 * @brief Contains data structures used by collision constraints
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
 * @version TODO
 * @bug Only Discrete Evaluator is implemented
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

#include <trajopt_ifopt/constraints/collision_types.h>

namespace trajopt_ifopt
{
CollisionCoeffData::CollisionCoeffData(const double& default_collision_coeff)
  : default_collision_coeff_(default_collision_coeff)
{
}

void CollisionCoeffData::setPairCollisionMarginData(const std::string& obj1,
                                                    const std::string& obj2,
                                                    const double& collision_coeff)
{
  auto key = tesseract_common::makeOrderedLinkPair(obj1, obj2);
  lookup_table_[key] = collision_coeff;
}

const double& CollisionCoeffData::getPairCollisionCoeff(const std::string& obj1, const std::string& obj2) const
{
  auto key = tesseract_common::makeOrderedLinkPair(obj1, obj2);
  const auto it = lookup_table_.find(key);

  if (it != lookup_table_.end())
  {
    return it->second;
  }
  return default_collision_coeff_;
}

TrajOptCollisionConfig::TrajOptCollisionConfig(double margin, double coeff)
  : CollisionCheckConfig(margin), collision_coeff_data(coeff)
{
}

GradientResults::GradientResults(const Eigen::Vector3d& data) : data(data) {}

}  // namespace trajopt_ifopt
