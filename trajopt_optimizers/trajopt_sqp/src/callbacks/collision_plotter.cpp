/**
 * @file collision_plotter.cpp
 * @brief Plots collision results
 *
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug This has not been implemented
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
#include <trajopt_sqp/callbacks/collision_plotter.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <trajopt/typedefs.hpp>

using namespace trajopt_sqp;

CollisionPlottingCallback::CollisionPlottingCallback(tesseract_visualization::Visualization::Ptr plotter)
  : plotter_(std::move(plotter))
{
}

void CollisionPlottingCallback::plot(const ifopt::Problem& /*nlp*/)
{
  std::cout << "Collision plotting has not been implemented. PRs welcome" << std::endl;
}

void CollisionPlottingCallback::addConstraintSet(
    const trajopt::DiscreteCollisionConstraintIfopt::ConstPtr& collision_constraint)
{
  collision_constraints_.push_back(collision_constraint);
}

void CollisionPlottingCallback::addConstraintSet(
    const std::vector<trajopt::DiscreteCollisionConstraintIfopt::ConstPtr>& collision_constraints)
{
  for (const auto& cnt : collision_constraints)
    collision_constraints_.push_back(cnt);
}

bool CollisionPlottingCallback::execute(const ifopt::Problem& nlp, const trajopt_sqp::SQPResults&)
{
  plot(nlp);
  return true;
}
