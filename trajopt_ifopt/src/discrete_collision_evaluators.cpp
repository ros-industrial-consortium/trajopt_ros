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

#include <trajopt_ifopt/constraints/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision_utils.h>

namespace trajopt
{
SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(
    tesseract_kinematics::ForwardKinematics::ConstPtr manip,
    tesseract_environment::Environment::ConstPtr env,
    tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
    const Eigen::Isometry3d& world_to_base,
    TrajOptCollisionConfig::ConstPtr collision_config,
    bool dynamic_environment)
  : manip_(std::move(manip))
  , env_(std::move(env))
  , adjacency_map_(std::move(adjacency_map))
  , world_to_base_(world_to_base)
  , collision_config_(std::move(collision_config))
  , state_solver_(env_->getStateSolver())
  , dynamic_environment_(dynamic_environment)
{
  // If the environment is not expected to change, then the cloned state solver may be used each time.
  if (dynamic_environment_)
  {
    get_state_fn_ = [&](const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
      return env_->getState(joint_names, joint_values);
    };
  }
  else
  {
    get_state_fn_ = [&](const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
      return state_solver_->getState(joint_names, joint_values);
    };
  }

  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setActiveCollisionObjects(adjacency_map_->getActiveLinkNames());
  contact_manager_->setCollisionMarginData(collision_config_->collision_margin_data);
  // Increase the default by the buffer
  contact_manager_->setDefaultCollisionMarginData(collision_config_->collision_margin_data.getMaxCollisionMargin() +
                                                  collision_config_->collision_margin_buffer);
}

CollisionCacheData::ConstPtr
SingleTimestepCollisionEvaluator::CalcCollisions(const Eigen::Ref<const Eigen::VectorXd>& dof_vals)
{
  size_t key = getHash(dof_vals);
  auto it = m_cache.get(key);
  if (it != nullptr)
  {
    CONSOLE_BRIDGE_logDebug("Using cached collision check");
    return *it;
  }

  CONSOLE_BRIDGE_logDebug("Not using cached collision check");
  auto data = std::make_shared<CollisionCacheData>();
  CalcCollisionsHelper(dof_vals, data->contact_results_map);
  tesseract_collision::flattenCopyResults(data->contact_results_map, data->contact_results_vector);
  data->gradient_results_set.results.reserve(data->contact_results_vector.size());
  data->gradient_results_set.dof = static_cast<int>(dof_vals.size());
  for (tesseract_collision::ContactResult& dist_result : data->contact_results_vector)
  {
    GradientResults result = GetGradient(dof_vals, dist_result);
    data->gradient_results_set.num_equations += (result.gradients[0].has_gradient + result.gradients[1].has_gradient);
    data->gradient_results_set.add(result);
  }
  m_cache.put(key, data);
  return data;
}

void SingleTimestepCollisionEvaluator::CalcCollisionsHelper(const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                                                            tesseract_collision::ContactResultMap& dist_results)
{
  tesseract_environment::EnvState::Ptr state = get_state_fn_(manip_->getJointNames(), dof_vals);

  for (const auto& link_name : env_->getActiveLinkNames())
    contact_manager_->setCollisionObjectsTransform(link_name, state->link_transforms[link_name]);

  contact_manager_->contactTest(dist_results, collision_config_->contact_request);

  for (auto& pair : dist_results)
  {
    // Contains the contact distance threshold and coefficient for the given link pair
    double dist = collision_config_->collision_margin_data.getPairCollisionMargin(pair.first.first, pair.first.second);
    double coeff = collision_config_->collision_coeff_data.getPairCollisionCoeff(pair.first.first, pair.first.second);
    const Eigen::Vector2d data = { dist, coeff };
    auto end = std::remove_if(
        pair.second.begin(), pair.second.end(), [&data, this](const tesseract_collision::ContactResult& r) {
          return (!((data[0] + collision_config_->collision_margin_buffer) > r.distance));
        });
    pair.second.erase(end, pair.second.end());
  }
}

GradientResults SingleTimestepCollisionEvaluator::GetGradient(const Eigen::VectorXd& dofvals,
                                                              const tesseract_collision::ContactResult& contact_result)
{
  // Contains the contact distance threshold and coefficient for the given link pair
  double dist = collision_config_->collision_margin_data.getPairCollisionMargin(contact_result.link_names[0],
                                                                                contact_result.link_names[1]);
  double coeff = collision_config_->collision_coeff_data.getPairCollisionCoeff(contact_result.link_names[0],
                                                                               contact_result.link_names[1]);
  const Eigen::Vector3d data = { dist, collision_config_->collision_margin_buffer, coeff };

  return getGradient(dofvals, contact_result, data, manip_, adjacency_map_, world_to_base_);
}

const TrajOptCollisionConfig& SingleTimestepCollisionEvaluator::GetCollisionConfig() const
{
  return *collision_config_;
}

}  // namespace trajopt
