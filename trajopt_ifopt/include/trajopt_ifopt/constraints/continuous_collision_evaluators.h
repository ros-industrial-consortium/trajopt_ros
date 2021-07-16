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

namespace trajopt_ifopt
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

  ContinuousCollisionEvaluator() = default;
  virtual ~ContinuousCollisionEvaluator() = default;
  ContinuousCollisionEvaluator(const ContinuousCollisionEvaluator&) = default;
  ContinuousCollisionEvaluator& operator=(const ContinuousCollisionEvaluator&) = default;
  ContinuousCollisionEvaluator(ContinuousCollisionEvaluator&&) = default;
  ContinuousCollisionEvaluator& operator=(ContinuousCollisionEvaluator&&) = default;

  /**
   * @brief Given joint names and values calculate the collision results for this evaluator
   * @param dof_vals0 Joint values for start state
   * @param dof_vals1 Joint values for end state
   * @return Collision cache data. If a cache does not exist for the provided joint values it evaluates and stores the
   * data.
   */
  virtual CollisionCacheData::ConstPtr CalcCollisionData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                         const Eigen::Ref<const Eigen::VectorXd>& dof_vals1) = 0;

  /**
   * @brief Extracts the gradient information based on the contact results for the transition between dof_vals0 and
   * dof_vals1
   * @param dof_vals0 Joint values for start state
   * @param dof_vals1 Joint values for end state
   * @param contact_results The contact results for transition between dof_vals0 and dof_vals1 to compute the gradient
   * data
   * @return The gradient results
   */
  virtual GradientResults CalcGradientData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                           const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                           const tesseract_collision::ContactResult& contact_results) = 0;

  /**
   * @brief Get the safety margin information.
   * @return Safety margin information
   */
  virtual const TrajOptCollisionConfig& GetCollisionConfig() const = 0;
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

  LVSContinuousCollisionEvaluator(std::shared_ptr<CollisionCache> collision_cache,
                                  tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                  tesseract_environment::Environment::ConstPtr env,
                                  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                  const Eigen::Isometry3d& world_to_base,
                                  TrajOptCollisionConfig::ConstPtr collision_config,
                                  bool dynamic_environment = false);

  CollisionCacheData::ConstPtr CalcCollisionData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                 const Eigen::Ref<const Eigen::VectorXd>& dof_vals1) override;

  GradientResults CalcGradientData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                   const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                   const tesseract_collision::ContactResult& contact_results) override;

  const TrajOptCollisionConfig& GetCollisionConfig() const override;

private:
  std::shared_ptr<CollisionCache> collision_cache_;
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  TrajOptCollisionConfig::ConstPtr collision_config_;
  tesseract_environment::StateSolver::Ptr state_solver_;
  GetStateFn get_state_fn_;
  bool dynamic_environment_;
  tesseract_collision::ContinuousContactManager::Ptr contact_manager_;

  CollisionCacheData::ConstPtr CalcCollisionsCacheDataHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                             const Eigen::Ref<const Eigen::VectorXd>& dof_vals1);

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

  LVSDiscreteCollisionEvaluator(std::shared_ptr<CollisionCache> collision_cache,
                                tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                tesseract_environment::Environment::ConstPtr env,
                                tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                const Eigen::Isometry3d& world_to_base,
                                TrajOptCollisionConfig::ConstPtr collision_config,
                                bool dynamic_environment = false);

  CollisionCacheData::ConstPtr CalcCollisionData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                 const Eigen::Ref<const Eigen::VectorXd>& dof_vals1) override;

  GradientResults CalcGradientData(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                   const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                                   const tesseract_collision::ContactResult& contact_results) override;

  const TrajOptCollisionConfig& GetCollisionConfig() const override;

private:
  std::shared_ptr<CollisionCache> collision_cache_;
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  TrajOptCollisionConfig::ConstPtr collision_config_;
  tesseract_environment::StateSolver::Ptr state_solver_;
  GetStateFn get_state_fn_;
  bool dynamic_environment_;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;

  CollisionCacheData::ConstPtr CalcCollisionsCacheDataHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                                                             const Eigen::Ref<const Eigen::VectorXd>& dof_vals1);

  void CalcCollisionsHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                            const Eigen::Ref<const Eigen::VectorXd>& dof_vals1,
                            tesseract_collision::ContactResultMap& dist_results);
};
}  // namespace trajopt_ifopt
#endif  // TRAJOPT_IFOPT_CONTINUOUS_COLLISION_EVALUATOR_H
