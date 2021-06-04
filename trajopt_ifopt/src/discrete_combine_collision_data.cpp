/**
 * @file discrete_combine_collision_data.cpp
 * @brief A container for function which combine collision data
 *
 * @author Levi Armstrong
 * @date June 4, 2021
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

#include <trajopt_ifopt/constraints/discrete_combine_collision_data.h>
#include <trajopt_ifopt/constraints/collision_utils.h>

namespace trajopt
{
DiscreteCombineCollisionData::DiscreteCombineCollisionData(CombineCollisionDataMethod method, long dof)
{
  switch (method)
  {
    case CombineCollisionDataMethod::SUM:
    {
      combine_values = [](const CollisionCacheData& collision_data) {
        return getSumValuesPost(collision_data.gradient_results_set);
      };

      combine_jacobian = [dof](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(dof);
        Eigen::VectorXd grad_vec = getSumGradientPost(collision_data.gradient_results_set, dof);

        // Collision is 1 x n_dof
        for (int j = 0; j < dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };

      break;
    }
    case CombineCollisionDataMethod::WEIGHTED_SUM:
    {
      combine_values = [](const CollisionCacheData& collision_data) {
        return getSumWeightedValuesPost(collision_data.gradient_results_set);
      };

      combine_jacobian = [dof](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(dof);
        Eigen::VectorXd grad_vec = getWeightedSumGradientPost(collision_data.gradient_results_set, dof);

        // Collision is 1 x n_dof
        for (int j = 0; j < dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };

      break;
    }
    case CombineCollisionDataMethod::AVERAGE:
    {
      combine_values = [](const CollisionCacheData& collision_data) {
        return getAverageValuesPost(collision_data.gradient_results_set);
      };

      combine_jacobian = [dof](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(dof);
        Eigen::VectorXd grad_vec = getAvgGradientPost(collision_data.gradient_results_set, dof);

        // Collision is 1 x n_dof
        for (int j = 0; j < dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };

      break;
    }
    case CombineCollisionDataMethod::WEIGHTED_AVERAGE:
    {
      combine_values = [](const CollisionCacheData& collision_data) {
        return getAverageWeightedValuesPost(collision_data.gradient_results_set);
      };

      combine_jacobian = [dof](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(dof);
        Eigen::VectorXd grad_vec = getWeightedAvgGradientPost(collision_data.gradient_results_set, dof);

        // Collision is 1 x n_dof
        for (int j = 0; j < dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      break;
    }
    case CombineCollisionDataMethod::LEAST_SQUARES:
    {
      combine_values = [](const CollisionCacheData& collision_data) {
        return getAverageValuesPost(collision_data.gradient_results_set);
      };

      combine_jacobian = [dof](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(dof);
        Eigen::VectorXd grad_vec = getLeastSquaresGradientPost(collision_data.gradient_results_set, dof);

        // Collision is 1 x n_dof
        for (int j = 0; j < dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      break;
    }
    case CombineCollisionDataMethod::WEIGHTED_LEAST_SQUARES:
    {
      combine_values = [](const CollisionCacheData& collision_data) {
        return getAverageWeightedValuesPost(collision_data.gradient_results_set);
      };

      combine_jacobian = [dof](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(dof);
        Eigen::VectorXd grad_vec = getWeightedLeastSquaresGradientPost(collision_data.gradient_results_set, dof);

        // Collision is 1 x n_dof
        for (int j = 0; j < dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      break;
    }
  }
}
}  // namespace trajopt
