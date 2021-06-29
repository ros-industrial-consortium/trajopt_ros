/**
 * @file continuous_combine_collision_data.cpp
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

#include <trajopt_ifopt/constraints/continuous_combine_collision_data.h>
#include <trajopt_ifopt/constraints/collision_utils.h>

namespace trajopt_ifopt
{
ContinuousCombineCollisionData::ContinuousCombineCollisionData(CombineCollisionDataMethod method)
{
  switch (method)
  {
    case CombineCollisionDataMethod::SUM:
    {
      combine_values_prev = [](const CollisionCacheData& collision_data) {
        return getSumValuesPrev(collision_data.gradient_results_set);
      };
      combine_values_post = [](const CollisionCacheData& collision_data) {
        return getSumValuesPost(collision_data.gradient_results_set);
      };
      combine_values_cent = [](const CollisionCacheData& collision_data_prev,
                               const CollisionCacheData& collision_data_post) {
        return getSumValuesCent(collision_data_prev.gradient_results_set, collision_data_post.gradient_results_set);
      };

      combine_jacobian_prev = [](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getSumGradientPost(collision_data.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      combine_jacobian_post = [](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getSumGradientPost(collision_data.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      combine_jacobian_cent = [](ifopt::Component::Jacobian& jac_block,
                                 const CollisionCacheData& collision_data_prev,
                                 const CollisionCacheData& collision_data_post) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data_prev.gradient_results_set.dof);
        Eigen::VectorXd grad_vec =
            getSumGradientCent(collision_data_prev.gradient_results_set, collision_data_post.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data_prev.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      break;
    }
    case CombineCollisionDataMethod::WEIGHTED_SUM:
    {
      combine_values_prev = [](const CollisionCacheData& collision_data) {
        return getSumWeightedValuesPrev(collision_data.gradient_results_set);
      };
      combine_values_post = [](const CollisionCacheData& collision_data) {
        return getSumWeightedValuesPost(collision_data.gradient_results_set);
      };
      combine_values_cent = [](const CollisionCacheData& collision_data_prev,
                               const CollisionCacheData& collision_data_post) {
        return getSumWeightedValuesCent(collision_data_prev.gradient_results_set,
                                        collision_data_post.gradient_results_set);
      };

      combine_jacobian_prev = [](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getWeightedSumGradientPrev(collision_data.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      combine_jacobian_post = [](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getWeightedSumGradientPost(collision_data.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      combine_jacobian_cent = [](ifopt::Component::Jacobian& jac_block,
                                 const CollisionCacheData& collision_data_prev,
                                 const CollisionCacheData& collision_data_post) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data_prev.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getWeightedSumGradientCent(collision_data_prev.gradient_results_set,
                                                              collision_data_post.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data_prev.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };

      break;
    }
    case CombineCollisionDataMethod::AVERAGE:
    {
      combine_values_prev = [](const CollisionCacheData& collision_data) {
        return getAverageValuesPrev(collision_data.gradient_results_set);
      };
      combine_values_post = [](const CollisionCacheData& collision_data) {
        return getAverageValuesPost(collision_data.gradient_results_set);
      };
      combine_values_cent = [](const CollisionCacheData& collision_data_prev,
                               const CollisionCacheData& collision_data_post) {
        return getAverageValuesCent(collision_data_prev.gradient_results_set, collision_data_post.gradient_results_set);
      };

      combine_jacobian_prev = [](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getAvgGradientPrev(collision_data.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      combine_jacobian_post = [](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getAvgGradientPost(collision_data.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      combine_jacobian_cent = [](ifopt::Component::Jacobian& jac_block,
                                 const CollisionCacheData& collision_data_prev,
                                 const CollisionCacheData& collision_data_post) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data_prev.gradient_results_set.dof);
        Eigen::VectorXd grad_vec =
            getAvgGradientCent(collision_data_prev.gradient_results_set, collision_data_post.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data_prev.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      break;
    }
    case CombineCollisionDataMethod::WEIGHTED_AVERAGE:
    {
      combine_values_prev = [](const CollisionCacheData& collision_data) {
        return getAverageWeightedValuesPrev(collision_data.gradient_results_set);
      };
      combine_values_post = [](const CollisionCacheData& collision_data) {
        return getAverageWeightedValuesPost(collision_data.gradient_results_set);
      };
      combine_values_cent = [](const CollisionCacheData& collision_data_prev,
                               const CollisionCacheData& collision_data_post) {
        return getAverageWeightedValuesCent(collision_data_prev.gradient_results_set,
                                            collision_data_post.gradient_results_set);
      };

      combine_jacobian_prev = [](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getWeightedAvgGradientPrev(collision_data.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      combine_jacobian_post = [](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getWeightedAvgGradientPost(collision_data.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      combine_jacobian_cent = [](ifopt::Component::Jacobian& jac_block,
                                 const CollisionCacheData& collision_data_prev,
                                 const CollisionCacheData& collision_data_post) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data_prev.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getWeightedAvgGradientCent(collision_data_prev.gradient_results_set,
                                                              collision_data_post.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data_prev.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      break;
    }
    case CombineCollisionDataMethod::LEAST_SQUARES:
    {
      combine_values_prev = [](const CollisionCacheData& collision_data) {
        return getAverageValuesPrev(collision_data.gradient_results_set);
      };
      combine_values_post = [](const CollisionCacheData& collision_data) {
        return getAverageValuesPost(collision_data.gradient_results_set);
      };
      combine_values_cent = [](const CollisionCacheData& collision_data_prev,
                               const CollisionCacheData& collision_data_post) {
        return getAverageValuesCent(collision_data_prev.gradient_results_set, collision_data_post.gradient_results_set);
      };

      combine_jacobian_prev = [](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getLeastSquaresGradientPrev(collision_data.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      combine_jacobian_post = [](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getLeastSquaresGradientPost(collision_data.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      combine_jacobian_cent = [](ifopt::Component::Jacobian& jac_block,
                                 const CollisionCacheData& collision_data_prev,
                                 const CollisionCacheData& collision_data_post) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data_prev.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getLeastSquaresGradientCent(collision_data_prev.gradient_results_set,
                                                               collision_data_post.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data_prev.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      break;
    }
    case CombineCollisionDataMethod::WEIGHTED_LEAST_SQUARES:
    {
      combine_values_prev = [](const CollisionCacheData& collision_data) {
        return getAverageWeightedValuesPrev(collision_data.gradient_results_set);
      };
      combine_values_post = [](const CollisionCacheData& collision_data) {
        return getAverageWeightedValuesPost(collision_data.gradient_results_set);
      };
      combine_values_cent = [](const CollisionCacheData& collision_data_prev,
                               const CollisionCacheData& collision_data_post) {
        return getAverageWeightedValuesCent(collision_data_prev.gradient_results_set,
                                            collision_data_post.gradient_results_set);
      };

      combine_jacobian_prev = [](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getWeightedLeastSquaresGradientPrev(collision_data.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      combine_jacobian_post = [](ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getWeightedLeastSquaresGradientPost(collision_data.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      combine_jacobian_cent = [](ifopt::Component::Jacobian& jac_block,
                                 const CollisionCacheData& collision_data_prev,
                                 const CollisionCacheData& collision_data_post) {
        // Reserve enough room in the sparse matrix
        jac_block.reserve(collision_data_prev.gradient_results_set.dof);
        Eigen::VectorXd grad_vec = getWeightedLeastSquaresGradientCent(collision_data_prev.gradient_results_set,
                                                                       collision_data_post.gradient_results_set);

        // Collision is 1 x n_dof
        for (int j = 0; j < collision_data_prev.gradient_results_set.dof; j++)
          jac_block.coeffRef(0, j) = -1.0 * grad_vec[j];
      };
      break;
    }
  }
}
}  // namespace trajopt_ifopt
