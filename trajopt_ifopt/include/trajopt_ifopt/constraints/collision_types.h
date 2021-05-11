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
#include <tesseract_collision/core/types.h>
#include <tesseract_environment/core/environment.h>
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
 */
enum class GradientCombineMethod
{
  SUM = 0,
  WEIGHTED_SUM = 1,
  LEAST_SQUARES = 2,
  WEIGHTED_LEAST_SQUARES = 3
};

/**
 * @brief This contains the different types of expression evaluators used when performing continuous collision checking.
 */
enum class ContinuousCollisionEvaluatorType
{
  /** @brief Both start and end state variables are free to be adjusted */
  START_FREE_END_FREE = 0,

  /** @brief Only start state variables are free to be adjusted */
  START_FREE_END_FIXED = 1,

  /** @brief Only end state variables are free to be adjusted */
  START_FIXED_END_FREE = 2,

  /**
   * @brief Both start and end state variables are free to be adjusted
   * The jacobian is calculated using a weighted sum over the interpolated states for a given link pair.
   */
  START_FREE_END_FREE_WEIGHTED_SUM = 3,
  /**
   * @brief Only start state variables are free to be adjusted
   * The jacobian is calculated using a weighted sum over the interpolated states for a given link pair.
   */
  START_FREE_END_FIXED_WEIGHTED_SUM = 4,
  /**
   * @brief Only end state variables are free to be adjusted
   * The jacobian is calculated using a weighted sum over the interpolated states for a given link pair.
   */
  START_FIXED_END_FREE_WEIGHTED_SUM = 5,
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

  /** @brief The gradient results data for LinkA and LinkB */
  std::array<LinkGradientResults, 2> gradients;

  /** @brief The error std::max<double>(((dist - dist_result.distance) * coeff), 0.) */
  double error {0};

  /** @brief The error std::max<double>(((dist + buffer - dist_result.distance) * coeff), 0.) */
  double error_with_buffer {0};

  /**
   * @brief The link pair safety margin data
   * [0] safety margin
   * [1] safety margin buffer
   * [2] coefficent
   */
  Eigen::Vector3d data;
};
}  // namespace trajopt
#endif  // TRAJOPT_IFOPT_COLLISION_TYPES_H
