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
#ifndef TRAJOPT_IFOPT_COLLISION_TYPES_H
#define TRAJOPT_IFOPT_COLLISION_TYPES_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <array>
#include <memory>
#include <tesseract_collision/core/types.h>
#include <tesseract_environment/core/environment.h>
#include <ifopt/composite.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt
{
using GetStateFn =
    std::function<tesseract_environment::EnvState::Ptr(const std::vector<std::string>& joint_names,
                                                       const Eigen::Ref<const Eigen::VectorXd>& joint_values)>;

/**
 * @brief IFOPT does not support dynamically changing the constraint size so
 * all collision gradient data must be combined into a single gradient and error.
 * This allows switching between different methods for calculating a single gradient and error.
 * @note Currently the WEIGHTED_AVERAGE method works the best, but may change after further testing.
 */
enum class CombineCollisionDataMethod
{
  SUM = 0,
  WEIGHTED_SUM = 1,
  AVERAGE = 2,
  WEIGHTED_AVERAGE = 3,
  LEAST_SQUARES = 4,
  WEIGHTED_LEAST_SQUARES = 5
};

/** @brief Stores information about how the margins allowed between collision objects */
struct CollisionCoeffData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CollisionCoeffData>;
  using ConstPtr = std::shared_ptr<const CollisionCoeffData>;

  CollisionCoeffData(const double& default_collision_coeff = 1);

  /**
   * @brief Set the margin for a given contact pair
   *
   * The order of the object names does not matter, that is handled internal to
   * the class.
   *
   * @param obj1 The first object name. Order doesn't matter
   * @param obj2 The Second object name. Order doesn't matter
   * @param Coeff used for collision pair
   */
  void setPairCollisionMarginData(const std::string& obj1, const std::string& obj2, const double& collision_coeff);

  /**
   * @brief Get the pairs collision coeff
   *
   * If a collision coeff for the request pair does not exist it returns the default collision coeff
   *
   * @param obj1 The first object name
   * @param obj2 The second object name
   * @return Coefficient
   */
  const double& getPairCollisionCoeff(const std::string& obj1, const std::string& obj2) const;

private:
  /// Stores the collision margin used if no pair-specific one is set
  double default_collision_coeff_;

  /// A map of link pair names to contact distance
  std::unordered_map<tesseract_common::LinkNamesPair, double, tesseract_common::PairHash> lookup_table_;
};

/**
 * @brief Config settings for collision terms.
 */
struct TrajOptCollisionConfig : public tesseract_collision::CollisionCheckConfig
{
  using Ptr = std::shared_ptr<TrajOptCollisionConfig>;
  using ConstPtr = std::shared_ptr<const TrajOptCollisionConfig>;

  TrajOptCollisionConfig() = default;
  TrajOptCollisionConfig(double margin, double coeff);

  /** @brief The collision coeff/weight */
  CollisionCoeffData collision_coeff_data;

  /** @brief Additional collision margin that is added for the collision check but is not used when calculating the
   * error */
  double collision_margin_buffer{ 0 };
};

/** @brief A data structure to contain a links gradient results */
struct LinkGradientResults
{
  /** @brief Indicates if gradient results are available */
  bool has_gradient{ false };

  /** @brief Gradient Results */
  Eigen::VectorXd gradient;

  /** @brief The minimum translation vector to move link out of collision*/
  Eigen::VectorXd translation_vector;

  /** @brief The robot jocobian at the contact point */
  Eigen::MatrixXd jacobian;

  /** @brief Gradient Scale */
  double scale{ 1.0 };
};

/** @brief A data structure to contain a link pair gradient results */
struct GradientResults
{
  /**
   * @brief Construct the GradientResults
   * @param data The link pair safety margin data
   */
  GradientResults(const Eigen::Vector3d& data);

  /**
   * @brief The gradient results data for LinkA and LinkB
   * @details This is used by both discrete and continuous collision checking.
   * In the case of continuous collision checking this is the gradient at timestep0
   */
  std::array<LinkGradientResults, 2> gradients;

  /**
   * @brief The gradient results data for LinkA and LinkB
   * @details In the case of continuous collision checking, this is used to store the gradient at timestep1.
   */
  std::array<LinkGradientResults, 2> cc_gradients;

  /** @brief The error (dist - dist_result.distance) */
  double error{ 0 };

  /** @brief The error (dist + buffer - dist_result.distance) */
  double error_with_buffer{ 0 };

  /**
   * @brief The link pair safety margin data
   * [0] safety margin
   * [1] safety margin buffer
   * [2] coefficent
   */
  Eigen::Vector3d data;
};

/** @brief A set of gradient results */
struct GradientResultsSet
{
  GradientResultsSet() = default;
  GradientResultsSet(std::size_t reserve) { results.reserve(reserve); }

  /** @brief The number of dof */
  int dof{ -1 };

  /** @brief The collision margin buffer used when calculating the results */
  double collision_margin_buffer{ 0 };

  /**
   * @brief Indicate if this set is from a continuous contact checker
   * @details If false, the data is from a discrete contact checker
   */
  bool is_continuous{ false };

  /** @brief The max error in the gradient_results */
  double max_error{ 0 };

  /** @brief The max error with buffer in the gradient_results */
  double max_error_with_buffer{ 0 };

  /** @brief The max weighted error (error * coeff) in the gradient_results */
  double max_weighted_error{ 0 };

  /** @brief The max weighted error (error * coeff) with buffer in the gradient_results */
  double max_weighted_error_with_buffer{ 0 };

  /**
   * @brief The number of equations that would be generated
   * @details Each GradientResults has the potential to generate one or two equations
   * In the case of continuous collision checking this is the number of equations at timestep0
   */
  long num_equations{ 0 };

  /**
   * @brief The number of equations that would be generated
   * @details Each GradientResults has the potential to generate one or two equations
   * This is used by both discrete and continuous collision checking.
   * In the case of continuous collision checking this is the number of equations at timestep0
   */
  long cc_num_equations{ 0 };

  /** @brief The stored gradient results for this set */
  std::vector<GradientResults> results;

  void add(const GradientResults& gradient_result)
  {
    if (gradient_result.error > max_error)
      max_error = gradient_result.error;

    if (gradient_result.error_with_buffer > max_error_with_buffer)
      max_error_with_buffer = gradient_result.error_with_buffer;

    double we = gradient_result.error * gradient_result.data[2];
    if (we > max_weighted_error)
      max_weighted_error = we;

    we = gradient_result.error_with_buffer * gradient_result.data[2];
    if (we > max_weighted_error_with_buffer)
      max_weighted_error_with_buffer = we;

    results.push_back(gradient_result);
  }
};

/** @brief The data structure used to cache collision results data for discrete collision evaluator */
struct CollisionCacheData
{
  using Ptr = std::shared_ptr<CollisionCacheData>;
  using ConstPtr = std::shared_ptr<const CollisionCacheData>;

  tesseract_collision::ContactResultMap contact_results_map;
  tesseract_collision::ContactResultVector contact_results_vector;
  GradientResultsSet gradient_results_set;
};

using CombineValuesPrevFn = std::function<Eigen::VectorXd(const CollisionCacheData& collision_data_prev)>;
using CombineValuesCentFn = std::function<Eigen::VectorXd(const CollisionCacheData& collision_data_prev,
                                                          const CollisionCacheData& collision_data_post)>;
using CombineValuesPostFn = std::function<Eigen::VectorXd(const CollisionCacheData& collision_data_post)>;

using CombineJacobianPrevFn =
    std::function<void(ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data_prev)>;
using CombineJacobianPostFn =
    std::function<void(ifopt::Component::Jacobian& jac_block, const CollisionCacheData& collision_data_post)>;
using CombineJacobianCentFn = std::function<void(ifopt::Component::Jacobian& jac_block,
                                                 const CollisionCacheData& collision_data_prev,
                                                 const CollisionCacheData& collision_data_post)>;

}  // namespace trajopt
#endif  // TRAJOPT_IFOPT_COLLISION_TYPES_H
