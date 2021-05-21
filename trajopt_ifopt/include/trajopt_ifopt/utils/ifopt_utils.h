/**
 * @file ifopt_utils.h
 * @brief Contains utilities for converting IFOPT types to other types
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
#ifndef TRAJOPT_IFOPT_IFOPT_UTILS_H
#define TRAJOPT_IFOPT_IFOPT_UTILS_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <ifopt/cost_term.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt
{
/**
 * @brief Converts a MatrixX2d (e.g. from forward_kinematics->getLimits()) to a vector of ifopt Bounds
 * @param limits MatrixX2d of bounds. Column 0 will be lower bound. Column 1 will be upper bound
 * @return Vector of ifopt::Bounds
 */
inline std::vector<ifopt::Bounds> toBounds(const Eigen::Ref<const Eigen::MatrixX2d>& limits)
{
  std::vector<ifopt::Bounds> bounds;
  for (Eigen::Index i = 0; i < limits.rows(); i++)
    bounds.emplace_back(limits(i, 0), limits(i, 1));
  return bounds;
}

/**
 * @brief Converts 2 VectorXd to a vector of ifopt Bounds
 * @param lower_limits Lower limits
 * @param upper_limits Upper limits
 * @return  Vector of ifopt::Bounds
 */
inline std::vector<ifopt::Bounds> toBounds(const Eigen::Ref<const Eigen::VectorXd>& lower_limits,
                                           const Eigen::Ref<const Eigen::VectorXd>& upper_limits)
{
  assert(lower_limits.size() == upper_limits.size());
  Eigen::MatrixX2d limits(lower_limits.rows(), 2);
  limits.col(0) = lower_limits;
  limits.col(1) = upper_limits;
  return toBounds(limits);
}

/**
 * @brief Interpolates between two Eigen::VectorXds
 * @param start Start vector. This will be the first vector in the results
 * @param end End vector. This will be the last vector in the results
 * @param steps Length of the returned vector
 * @return A vector of Eigen vectors interpolated from start to end
 */
inline std::vector<Eigen::VectorXd> interpolate(const Eigen::Ref<const Eigen::VectorXd>& start,
                                                const Eigen::Ref<const Eigen::VectorXd>& end,
                                                Eigen::Index steps)
{
  assert(start.size() == end.size());
  Eigen::VectorXd delta = (end - start) / static_cast<double>(steps - 1);
  Eigen::VectorXd running = start;
  std::vector<Eigen::VectorXd> results;
  for (Eigen::Index i = 0; i < steps; i++)
  {
    results.push_back(running);
    running += delta;
  }
  return results;
}

/**
 * @brief Gets the closes vector to the input given the bounds
 * @param input Input vector
 * @param bounds Bounds on that vector
 * @return Output vector. If input is outside a bound, force it to the boundary
 */
inline Eigen::VectorXd getClosestValidPoint(const Eigen::Ref<const Eigen::VectorXd>& input,
                                            const std::vector<ifopt::Bounds>& bounds)
{
  // Convert Bounds to VectorXds
  Eigen::VectorXd bound_lower(static_cast<Eigen::Index>(bounds.size()));
  Eigen::VectorXd bound_upper(static_cast<Eigen::Index>(bounds.size()));
  for (std::size_t i = 0; i < bounds.size(); i++)
  {
    bound_lower[static_cast<Eigen::Index>(i)] = bounds[i].lower_;
    bound_upper[static_cast<Eigen::Index>(i)] = bounds[i].upper_;
  }

  // If input is outside a bound, force it to the boundary
  Eigen::VectorXd valid_point(static_cast<Eigen::Index>(bounds.size()));
  valid_point = input.cwiseMax(bound_lower);
  valid_point = valid_point.cwiseMin(bound_upper);
  return valid_point;
}

/**
 * @brief Calculate errors of the bounds
 * @param input The input values
 * @param bounds The bounds
 * @return The error given the bounds, if within the bounds it will be zero
 */
inline Eigen::VectorXd calcBoundsErrors(const Eigen::Ref<const Eigen::VectorXd>& input,
                                        const std::vector<ifopt::Bounds>& bounds)
{
  assert(input.rows() == static_cast<Eigen::Index>(bounds.size()));

  // Convert constraint bounds to VectorXd
  Eigen::ArrayXd bound_lower(input.rows());
  Eigen::ArrayXd bound_upper(input.rows());
  for (std::size_t i = 0; i < bounds.size(); i++)
  {
    bound_lower[static_cast<Eigen::Index>(i)] = bounds[i].lower_;
    bound_upper[static_cast<Eigen::Index>(i)] = bounds[i].upper_;
  }

  // Values will be negative if they violate the constrain
  Eigen::ArrayXd zero = Eigen::ArrayXd::Zero(input.rows());
  Eigen::ArrayXd dist_from_lower = (input.array() - bound_lower).min(zero);
  Eigen::ArrayXd dist_from_upper = (input.array() - bound_upper).max(zero);
  Eigen::ArrayXd worst_error = (dist_from_upper.abs() > dist_from_lower.abs()).select(dist_from_upper, dist_from_lower);

  return worst_error;
}

/**
 * @brief The absolute value of the Bounds Errors
 * @param input The input values
 * @param bounds The bounds
 * @return The absolute errors given the bounds, if within the bounds it will be zero
 */
inline Eigen::VectorXd calcBoundsViolations(const Eigen::Ref<const Eigen::VectorXd>& input,
                                            const std::vector<ifopt::Bounds>& bounds)
{
  return calcBoundsErrors(input, bounds).cwiseAbs();
}

}  // namespace trajopt
#endif
