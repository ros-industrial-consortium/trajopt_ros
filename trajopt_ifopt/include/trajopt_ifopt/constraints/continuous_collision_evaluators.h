/**
 * @file continuous_collision_evaluators.h
 * @brief Contains continuous evaluators for the collision constraint
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

#ifndef TRAJOPT_IFOPT_CONTINUOUS_COLLISION_EVALUATOR_H
#define TRAJOPT_IFOPT_CONTINUOUS_COLLISION_EVALUATOR_H

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
 * @brief This collision evaluator operates on two states and checks for collision between the two states using a
 * casted collision objects between to intermediate interpolated states.
 */
class ContinuousCollisionEvaluator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ContinuousCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const ContinuousCollisionEvaluator>;

  virtual ~ContinuousCollisionEvaluator() = default;

  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals0 Joint values for state0
   * @param dof_vals1 Joint values for state1
   * @param dist_results Contact Results Map
   */
  virtual void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                              const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                              tesseract_collision::ContactResultMap& dist_results) = 0;

  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals0 Joint values for state0
   * @param dof_vals1 Joint values for state1
   * @param dist_results Contact Results Vector
   */
  virtual void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                              const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                              tesseract_collision::ContactResultVector& dist_results) = 0;

  /**
   * @brief Extracts the gradient information based on the contact results
   * @param dofvals The joint values
   * @param contact_result The contact results to compute the gradient
   * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
   * @return The gradient results
   */
  virtual GradientResults GetGradient(const Eigen::VectorXd& dofvals0,
                                      const Eigen::VectorXd& dofvals1,
                                      const tesseract_collision::ContactResult& contact_result,
                                      bool isTimestep1) = 0;

  /**
   * @brief Get the safety margin information.
   * @return Safety margin information
   */
  virtual TrajOptCollisionConfig& GetCollisionConfig() = 0;

  /**
   * @brief Get the evaluator type
   * @return The Evaluator Type
   */
  virtual ContinuousCollisionEvaluatorType GetEvaluatorType() const = 0;
};

/**
 * @brief This collision evaluator operates on two states and checks for collision between the two states using a
 * casted collision objects between to intermediate interpolated states.
 */
class LVSContinuousCollisionEvaluator : public ContinuousCollisionEvaluator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LVSContinuousCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const LVSContinuousCollisionEvaluator>;

  LVSContinuousCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                  tesseract_environment::Environment::ConstPtr env,
                                  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                  const Eigen::Isometry3d& world_to_base,
                                  const TrajOptCollisionConfig& collision_config,
                                  ContinuousCollisionEvaluatorType evaluator_type,
                                  bool dynamic_environment = false);

  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                      const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                      tesseract_collision::ContactResultMap& dist_results) override;

  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                      const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                      tesseract_collision::ContactResultVector& dist_results) override;

  GradientResults GetGradient(const Eigen::VectorXd& dofvals0,
                              const Eigen::VectorXd& dofvals1,
                              const tesseract_collision::ContactResult& contact_result,
                              bool isTimestep1) override;

  TrajOptCollisionConfig& GetCollisionConfig() override;

  ContinuousCollisionEvaluatorType GetEvaluatorType() const override;

  Cache<size_t, std::pair<tesseract_collision::ContactResultMap, tesseract_collision::ContactResultVector>, 10> m_cache;

private:
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  TrajOptCollisionConfig collision_config_;
  tesseract_environment::StateSolver::Ptr state_solver_;
  ContinuousCollisionEvaluatorType evaluator_type_;
  GetStateFn get_state_fn_;
  bool dynamic_environment_;
  tesseract_collision::ContinuousContactManager::Ptr contact_manager_;

  void CalcCollisionsHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                            tesseract_collision::ContactResultMap& dist_results);
};

/**
 * @brief This collision evaluator operates on two states and checks for collision between the two states using a
 * descrete collision objects at each intermediate interpolated states.
 */
class LVSDiscreteCollisionEvaluator : public ContinuousCollisionEvaluator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LVSDiscreteCollisionEvaluator>;
  using ConstPtr = std::shared_ptr<const LVSDiscreteCollisionEvaluator>;

  LVSDiscreteCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                tesseract_environment::Environment::ConstPtr env,
                                tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                const Eigen::Isometry3d& world_to_base,
                                const TrajOptCollisionConfig& collision_config,
                                ContinuousCollisionEvaluatorType evaluator_type,
                                bool dynamic_environment = false);

  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                      const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                      tesseract_collision::ContactResultMap& dist_results) override;

  void CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                      const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                      tesseract_collision::ContactResultVector& dist_results) override;

  GradientResults GetGradient(const Eigen::VectorXd& dofvals0,
                              const Eigen::VectorXd& dofvals1,
                              const tesseract_collision::ContactResult& contact_result,
                              bool isTimestep1) override;

  TrajOptCollisionConfig& GetCollisionConfig() override;

  ContinuousCollisionEvaluatorType GetEvaluatorType() const override;

  Cache<size_t, std::pair<tesseract_collision::ContactResultMap, tesseract_collision::ContactResultVector>, 10> m_cache;

private:
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  TrajOptCollisionConfig collision_config_;
  tesseract_environment::StateSolver::Ptr state_solver_;
  ContinuousCollisionEvaluatorType evaluator_type_;
  GetStateFn get_state_fn_;
  bool dynamic_environment_;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;

  void CalcCollisionsHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                            tesseract_collision::ContactResultMap& dist_results);
};
}  // namespace trajopt
#endif  // TRAJOPT_IFOPT_CONTINUOUS_COLLISION_EVALUATOR_H
