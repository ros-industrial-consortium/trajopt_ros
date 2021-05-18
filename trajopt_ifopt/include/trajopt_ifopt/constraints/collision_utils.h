/**
 * @file collision_utils.h
 * @brief Contains utility functions used by collision constraints
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
 * @version TODO
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
#ifndef TRAJOPT_IFOPT_COLLISION_UTILS_H
#define TRAJOPT_IFOPT_COLLISION_UTILS_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <tesseract_collision/core/types.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_environment/core/types.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision_types.h>

namespace trajopt
{
std::size_t getHash(const Eigen::Ref<const Eigen::VectorXd>& dof_vals);
std::size_t getHash(const Eigen::Ref<const Eigen::VectorXd>& dofvals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dofvals1);

/**
 * @brief This takes contacts results at each interpolated timestep and creates a single contact results map.
 * This also updates the cc_time and cc_type for the contact results
 * @param contacts_vector Contact results map at each interpolated timestep
 * @param contact_results The merged contact results map
 * @param dt delta time
 */
void processInterpolatedCollisionResults(std::vector<tesseract_collision::ContactResultMap>& contacts_vector,
                                         tesseract_collision::ContactResultMap& contact_results,
                                         const std::vector<std::string>& active_links,
                                         const TrajOptCollisionConfig& collision_config,
                                         ContinuousCollisionEvaluatorType evaluator_type,
                                         double dt);

/**
 * @brief Remove any results that are invalid.
 * Invalid state are contacts that occur at fixed states or have distances outside the threshold.
 * @param contact_results Contact results vector to process.
 */
void removeInvalidContactResults(tesseract_collision::ContactResultVector& contact_results,
                                 const Eigen::Vector3d& data,
                                 ContinuousCollisionEvaluatorType evaluator_type);

/**
 * @brief This takes a vector of gradient results and outputs a single gradient using least squares
 * @param grad_results A vector of gradient results
 * @param dof The DOF of the system
 * @param num_eq The number equations
 * @return A gradient
 */
Eigen::VectorXd getLeastSquaresGradient(std::vector<trajopt::GradientResults> grad_results, long dof, long num_eq);

/**
 * @brief This takes a vector of gradient results and outputs a single gradient using weighted least squares
 * where the weights are the error
 * @param grad_results A vector of gradient results
 * @param dof The DOF of the system
 * @param num_eq The number equations
 * @return A gradient
 */
Eigen::VectorXd getWeightedLeastSquaresGradient(std::vector<trajopt::GradientResults> grad_results,
                                                long dof,
                                                long num_eq);
Eigen::VectorXd getWeightedLeastSquaresGradient2(std::vector<trajopt::GradientResults> grad_results,
                                                 long dof,
                                                 long num_eq);

/** @brief These were from the original trajopt */
Eigen::VectorXd getAvgGradient(std::vector<trajopt::GradientResults> grad_results, long dof);
Eigen::VectorXd getWeightedAvgGradient(std::vector<trajopt::GradientResults> grad_results, long dof);
Eigen::VectorXd getScaledSumGradient(std::vector<trajopt::GradientResults> grad_results, long dof);
Eigen::VectorXd getSumGradient(std::vector<trajopt::GradientResults> grad_results, long dof);

/**
 * @brief Extracts the gradient information based on the contact results
 * @param dofvals The joint values
 * @param contact_result The contact results to compute the gradient
 * @param data Data associated with the link pair the contact results associated with.
 * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
 * @return The gradient results
 */
GradientResults getGradient(const Eigen::VectorXd& dofvals,
                            const tesseract_collision::ContactResult& contact_result,
                            const Eigen::Vector3d& data,
                            const tesseract_kinematics::ForwardKinematics::ConstPtr& manip,
                            const tesseract_environment::AdjacencyMap::ConstPtr& adjacency_map,
                            const Eigen::Isometry3d& world_to_base);

/**
 * @brief Extracts the gradient information based on the contact results
 * @param dofvals The joint values
 * @param contact_result The contact results to compute the gradient
 * @param data Data associated with the link pair the contact results associated with.
 * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
 * @return The gradient results
 */
GradientResults getGradient(const Eigen::VectorXd& dofvals0,
                            const Eigen::VectorXd& dofvals1,
                            const tesseract_collision::ContactResult& contact_result,
                            const Eigen::Vector3d& data,
                            const tesseract_kinematics::ForwardKinematics::ConstPtr& manip,
                            const tesseract_environment::AdjacencyMap::ConstPtr& adjacency_map,
                            const Eigen::Isometry3d& world_to_base,
                            bool isTimestep1);

void collisionsToDistances(const tesseract_collision::ContactResultVector& dist_results, std::vector<double>& dists);

void debugPrintInfo(const tesseract_collision::ContactResult& res,
                    const Eigen::VectorXd& dist_grad_A,
                    const Eigen::VectorXd& dist_grad_B,
                    const Eigen::VectorXd& dof_vals,
                    bool header = false);
}  // namespace trajopt
#endif  // TRAJOPT_IFOPT_COLLISION_UTILS_H
