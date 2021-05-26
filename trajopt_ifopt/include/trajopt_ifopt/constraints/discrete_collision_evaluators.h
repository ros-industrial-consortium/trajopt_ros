/**
 * @file discrete_collision_evaluators.h
 * @brief Contains discrete evaluators for the collision constraint
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
#ifndef TRAJOPT_IFOPT_DISCRETE_COLLISION_EVALUATOR_H
#define TRAJOPT_IFOPT_DISCRETE_COLLISION_EVALUATOR_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>

#include <tesseract_collision/core/types.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision_types.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt/cache.hxx>

namespace trajopt
{
/**
 * @brief This collision evaluator only operates on a single state in the trajectory and does not check for collisions
 * between states.
 */
class DiscreteCollisionEvaluator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<DiscreteCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const DiscreteCollisionEvaluator>;

  virtual ~DiscreteCollisionEvaluator() = default;

  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals Joint values set prior to collision checking
   * @param dist_results Contact Results Map
   */
  virtual void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                              tesseract_collision::ContactResultMap& dist_results) = 0;

  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals Joint values set prior to collision checking
   * @param dist_results Contact Results Vector
   */
  virtual void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                              tesseract_collision::ContactResultVector& dist_results) = 0;

  /**
   * @brief Get the safety margin information.
   * @return Safety margin information
   */
  virtual TrajOptCollisionConfig& GetCollisionConfig() = 0;

  /**
   * @brief Extracts the gradient information based on the contact results
   * @param dofvals The joint values
   * @param contact_result The contact results to compute the gradient
   * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
   * @return The gradient results
   */
  virtual GradientResults GetGradient(const Eigen::VectorXd& dofvals,
                                      const tesseract_collision::ContactResult& contact_result) = 0;
};

/**
 * @brief This collision evaluator only operates on a single state in the trajectory and does not check for collisions
 * between states.
 */
class SingleTimestepCollisionEvaluator : public DiscreteCollisionEvaluator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<SingleTimestepCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const SingleTimestepCollisionEvaluator>;

  SingleTimestepCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                   tesseract_environment::Environment::ConstPtr env,
                                   tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                   const Eigen::Isometry3d& world_to_base,
                                   const TrajOptCollisionConfig& collision_config,
                                   bool dynamic_environment = false);

  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                      tesseract_collision::ContactResultMap& dist_results) override;

  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                      tesseract_collision::ContactResultVector& dist_results) override;

  GradientResults GetGradient(const Eigen::VectorXd& dofvals,
                              const tesseract_collision::ContactResult& contact_result) override;

  TrajOptCollisionConfig& GetCollisionConfig() override;

  Cache<size_t, std::pair<tesseract_collision::ContactResultMap, tesseract_collision::ContactResultVector>, 10> m_cache;

private:
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  TrajOptCollisionConfig collision_config_;
  tesseract_environment::StateSolver::Ptr state_solver_;
  GetStateFn get_state_fn_;
  bool dynamic_environment_;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;

  void CalcCollisionsHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                            tesseract_collision::ContactResultMap& dist_results);
};

}  // namespace trajopt

#endif  // TRAJOPT_IFOPT_DISCRETE_COLLISION_EVALUATOR_H
