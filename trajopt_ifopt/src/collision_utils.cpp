/**
 * @file collision_utils.cpp
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

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/functional/hash.hpp>
#include <console_bridge/console.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision_utils.h>

namespace trajopt
{
std::size_t getHash(const Eigen::Ref<const Eigen::VectorXd>& dof_vals)
{
  std::size_t seed = 0;
  for (Eigen::Index i = 0; i < dof_vals.rows(); ++i)
    boost::hash_combine(seed, dof_vals[i]);

  return seed;
}

std::size_t getHash(const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals1)
{
  std::size_t seed = 0;
  for (Eigen::Index i = 0; i < dof_vals0.rows(); ++i)
  {
    boost::hash_combine(seed, dof_vals0[i]);
    boost::hash_combine(seed, dof_vals1[i]);
  }

  return seed;
}

void processInterpolatedCollisionResults(std::vector<tesseract_collision::ContactResultMap>& contacts_vector,
                                         tesseract_collision::ContactResultMap& contact_results,
                                         const std::vector<std::string>& active_links,
                                         const TrajOptCollisionConfig& collision_config,
                                         ContinuousCollisionEvaluatorType evaluator_type,
                                         double dt)
{
  // If contact is found the actual dt between the original two state must be recalculated based on where it
  // occured in the subtrajectory. Also the cc_type must also be recalculated but does not appear to be used
  // currently by trajopt.
  //  const std::vector<std::string>& active_links = adjacency_map_->getActiveLinkNames();
  for (size_t i = 0; i < contacts_vector.size(); ++i)
  {
    for (auto& pair : contacts_vector[i])
    {
      auto p = contact_results.find(pair.first);

      // Contains the contact distance threshold and coefficient for the given link pair
      double dist = collision_config.collision_margin_data.getPairCollisionMargin(pair.first.first, pair.first.second);
      double coeff = collision_config.collision_coeff_data.getPairCollisionCoeff(pair.first.first, pair.first.second);
      const Eigen::Vector3d data = { dist, collision_config.collision_margin_buffer, coeff };

      // Update cc_time and cc_type
      for (auto& r : pair.second)
      {
        // Iterate over the two time values in r.cc_time
        for (size_t j = 0; j < 2; ++j)
        {
          if (std::find(active_links.begin(), active_links.end(), r.link_names[j]) != active_links.end())
          {
            r.cc_time[j] = (r.cc_time[j] < 0) ? (static_cast<double>(i) * dt) :
                                                (static_cast<double>(i) * dt) + (r.cc_time[j] * dt);
            assert(r.cc_time[j] >= 0.0 && r.cc_time[j] <= 1.0);
            if (i == 0 && r.cc_type[j] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Time0;
            else if (i == (contacts_vector.size() - 1) &&
                     r.cc_type[j] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Time1;
            else
              r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Between;
          }
        }
      }

      // Dont include contacts at the fixed state
      removeInvalidContactResults(pair.second, data, evaluator_type);

      // If the contact pair does not exist in contact_results add it
      if (p == contact_results.end() && !pair.second.empty())
      {
        contact_results[pair.first] = pair.second;
      }
      else
      {
        // Note: Must include all contacts throughout the trajectory so the optimizer has all the information
        //      to understand how to adjust the start and end state to move it out of collision. Originally tried
        //      keeping the worst case only but ran into edge cases where this does not work in the units tests.

        // If it exists then add addition contacts to the contact_results pair
        for (auto& r : pair.second)
          p->second.push_back(r);
      }
    }
  }
}

void removeInvalidContactResults(tesseract_collision::ContactResultVector& contact_results,
                                 const Eigen::Vector3d& data,
                                 ContinuousCollisionEvaluatorType evaluator_type)
{
  auto end = std::remove_if(
      contact_results.begin(), contact_results.end(), [=, &data](const tesseract_collision::ContactResult& r) {
        switch (evaluator_type)
        {
          case ContinuousCollisionEvaluatorType::START_FREE_END_FREE:
          {
            break;
          }
          case ContinuousCollisionEvaluatorType::START_FIXED_END_FREE:
          {
            if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              return true;

            if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              return true;

            break;
          }
          case ContinuousCollisionEvaluatorType::START_FREE_END_FIXED:
          {
            if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              return true;

            if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              return true;

            break;
          }
          case ContinuousCollisionEvaluatorType::START_FREE_END_FREE_WEIGHTED_SUM:
          {
            break;
          }
          case ContinuousCollisionEvaluatorType::START_FIXED_END_FREE_WEIGHTED_SUM:
          {
            if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              return true;

            if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
              return true;

            break;
          }
          case ContinuousCollisionEvaluatorType::START_FREE_END_FIXED_WEIGHTED_SUM:
          {
            if (r.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              return true;

            if (r.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
              return true;

            break;
          }
          default:
          {
            PRINT_AND_THROW("Invalid CollisionExpressionEvaluatorType for "
                            "CollisionEvaluator::removeInvalidContactResults!");
          }
        };

        /** @todo Is this correct? (Levi)*/
        return (!((data[0] + data[1]) > r.distance));
      });

  contact_results.erase(end, contact_results.end());
}

Eigen::VectorXd getLeastSquaresGradient(std::vector<trajopt::GradientResults> grad_results, long dof, long num_eq)
{
  if (num_eq == 0)
    return Eigen::VectorXd::Zero(dof);

  Eigen::MatrixXd jacobian(3 * num_eq, dof);
  Eigen::VectorXd error(3 * num_eq);
  long cnt = 0;
  for (const auto& grad : grad_results)
  {
    if (grad.gradients[0].has_gradient)
    {
      long start_idx = cnt * 3;
      error.middleRows(start_idx, 3) = grad.error_with_buffer * grad.gradients[0].translation_vector;
      jacobian.middleRows(start_idx, 3) = grad.gradients[0].jacobian;
      cnt++;
    }
    if (grad.gradients[1].has_gradient)
    {
      long start_idx = cnt * 3;
      error.middleRows(start_idx, 3) = grad.error_with_buffer * grad.gradients[1].translation_vector;
      jacobian.middleRows(start_idx, 3) = grad.gradients[1].jacobian;
      cnt++;
    }
  }

  //  return jacobian.householderQr().solve(error);
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(dof);
  tesseract_kinematics::solvePInv(jacobian, error, grad_vec);

  return grad_vec.normalized();
  //  return jacobian.transpose() * error;
}

Eigen::VectorXd getWeightedLeastSquaresGradient(std::vector<trajopt::GradientResults> grad_results,
                                                long dof,
                                                long num_eq)
{
  if (num_eq == 0)
    return Eigen::VectorXd::Zero(dof);

  Eigen::MatrixXd jacobian(3 * num_eq, dof);
  Eigen::VectorXd error(3 * num_eq);
  long cnt = 0;
  for (const auto& grad : grad_results)
  {
    if (grad.gradients[0].has_gradient)
    {
      long start_idx = cnt * 3;
      error.middleRows(start_idx, 3) = grad.error_with_buffer * grad.gradients[0].translation_vector;
      jacobian.middleRows(start_idx, 3) = grad.gradients[0].jacobian;
      cnt++;
    }
    if (grad.gradients[1].has_gradient)
    {
      long start_idx = cnt * 3;
      error.middleRows(start_idx, 3) = grad.error_with_buffer * grad.gradients[1].translation_vector;
      jacobian.middleRows(start_idx, 3) = grad.gradients[1].jacobian;
      cnt++;
    }
  }
  // H=(A^T * W * A)^−1 * A^T * W so that y=Hb
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(dof);
  Eigen::MatrixXd weights = error.normalized().asDiagonal();
  Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
  tesseract_kinematics::solvePInv(
      jacobian_transpose * weights * jacobian, jacobian_transpose * weights * error, grad_vec);
  return grad_vec.normalized();

  //  return (jacobian_transpose * weights * jacobian).householderQr().solve(jacobian_transpose * weights * error);
}

Eigen::VectorXd getWeightedLeastSquaresGradient2(std::vector<trajopt::GradientResults> grad_results,
                                                 long dof,
                                                 long num_eq)
{
  if (num_eq == 0)
    return Eigen::VectorXd::Zero(dof);

  Eigen::MatrixXd jacobian(num_eq, dof);
  Eigen::VectorXd error(num_eq);
  long cnt = 0;
  for (const auto& grad : grad_results)
  {
    if (grad.gradients[0].has_gradient)
    {
      error(cnt) = grad.error_with_buffer;
      jacobian.row(cnt) = grad.gradients[0].gradient;
      //      long start_idx = cnt * 3;
      //      error.middleRows(start_idx, 3) = grad.error_with_buffer * grad.gradients[0].translation_vector;
      //      jacobian.middleRows(start_idx, 3) = grad.gradients[0].jacobian;
      cnt++;
    }
    if (grad.gradients[1].has_gradient)
    {
      error(cnt) = grad.error_with_buffer;
      jacobian.row(cnt) = grad.gradients[1].gradient;
      //      long start_idx = cnt * 3;
      //      error.middleRows(start_idx, 3) = grad.error_with_buffer * grad.gradients[1].translation_vector;
      //      jacobian.middleRows(start_idx, 3) = grad.gradients[1].jacobian;
      cnt++;
    }
  }
  // H=(A^T * W * A)^−1 * A^T * W so that y=Hb
  Eigen::MatrixXd weights = error.normalized().asDiagonal();
  Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
  //  return (jacobian_transpose * weights * jacobian).householderQr().solve(jacobian_transpose * weights * error);

  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(dof);
  tesseract_kinematics::solvePInv(
      jacobian_transpose * weights * jacobian, jacobian_transpose * weights * error, grad_vec);
  return grad_vec;
}

Eigen::VectorXd getAvgGradient(std::vector<trajopt::GradientResults> grad_results, long dof)
{
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(dof);
  if (grad_results.empty())
    return grad_vec;

  long cnt = 0;
  for (auto& grad : grad_results)
  {
    if (grad.gradients[0].has_gradient)
    {
      grad_vec += grad.error * grad.gradients[0].gradient;
      cnt++;
    }

    if (grad.gradients[1].has_gradient)
    {
      grad_vec += grad.error * grad.gradients[1].scale * grad.gradients[1].gradient;
      cnt++;
    }
  }
  return (grad_vec / cnt);
}

Eigen::VectorXd getWeightedAvgGradient(std::vector<trajopt::GradientResults> grad_results, long dof)
{
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(dof);
  if (grad_results.empty())
    return grad_vec;

  double total_weight = 0;
  for (auto& grad : grad_results)
  {
    if (grad.gradients[0].has_gradient)
    {
      total_weight += (grad.error * grad.gradients[0].scale);
      grad_vec += grad.error * grad.gradients[0].scale * grad.gradients[0].gradient;
    }

    if (grad.gradients[1].has_gradient)
    {
      total_weight += (grad.error * grad.gradients[1].scale);
      grad_vec += grad.error * grad.gradients[1].scale * grad.gradients[1].gradient;
    }
  }
  return (1.0 / total_weight) * grad_vec;
}

Eigen::VectorXd getScaledSumGradient(std::vector<trajopt::GradientResults> grad_results, long dof)
{
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(dof);
  for (auto& grad : grad_results)
  {
    if (grad.gradients[0].has_gradient)
      grad_vec += grad.gradients[0].scale * grad.gradients[0].gradient;
    if (grad.gradients[1].has_gradient)
      grad_vec += grad.gradients[1].scale * grad.gradients[1].gradient;
  }
  return grad_vec;
}

Eigen::VectorXd getSumGradient(std::vector<trajopt::GradientResults> grad_results, long dof)
{
  Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(dof);
  for (auto& grad : grad_results)
  {
    if (grad.gradients[0].has_gradient)
      grad_vec += grad.gradients[0].gradient;
    if (grad.gradients[1].has_gradient)
      grad_vec += grad.gradients[1].gradient;
  }
  return grad_vec;
}

void calcGradient(GradientResults& results,
                  std::size_t i,
                  const tesseract_environment::AdjacencyMapPair::ConstPtr& it,
                  const Eigen::VectorXd& dofvals,
                  const tesseract_collision::ContactResult& contact_result,
                  const tesseract_kinematics::ForwardKinematics::ConstPtr& manip,
                  const Eigen::Isometry3d& world_to_base,
                  bool isTimestep1)
{
  results.gradients[i].has_gradient = true;

  // Calculate Jacobian
  Eigen::MatrixXd jac = manip->calcJacobian(dofvals, it->link_name);

  // Need to change the base and ref point of the jacobian.
  // When changing ref point you must provide a vector from the current ref
  // point to the new ref point.
  results.gradients[i].scale = 1;
  Eigen::Isometry3d link_transform = contact_result.transform[i];
  if (contact_result.cc_type[i] != tesseract_collision::ContinuousCollisionType::CCType_None)
  {
    assert(contact_result.cc_time[i] >= 0.0 && contact_result.cc_time[i] <= 1.0);
    results.gradients[i].scale = (isTimestep1) ? contact_result.cc_time[i] : (1 - contact_result.cc_time[i]);
    link_transform = (isTimestep1) ? contact_result.cc_transform[i] : contact_result.transform[i];
  }
  tesseract_kinematics::jacobianChangeBase(jac, world_to_base);
  tesseract_kinematics::jacobianChangeRefPoint(jac,
                                               (link_transform * it->transform.inverse()).linear() *
                                                   (it->transform * contact_result.nearest_points_local[i]));

  //      Eigen::Isometry3d test_link_transform, temp1, temp2;
  //      manip_->calcFwdKin(test_link_transform, dofvals, it->link_name);
  //      temp1 = world_to_base_ * test_link_transform;
  //      temp2 = link_transform * it->transform.inverse();
  //      assert(temp1.isApprox(temp2, 0.0001));

  //      Eigen::MatrixXd jac_test;
  //      jac_test.resize(6, manip_->numJoints());
  //      tesseract_kinematics::numericalJacobian(jac_test, world_to_base_, *manip_, dofvals, it->link_name,
  //      contact_result.nearest_points_local[i]); bool check = jac.isApprox(jac_test, 1e-3); assert(check == true);

  results.gradients[i].translation_vector = ((i == 0) ? -1.0 : 1.0) * contact_result.normal;
  results.gradients[i].jacobian = jac.topRows(3);
  results.gradients[i].gradient = results.gradients[i].translation_vector.transpose() * results.gradients[i].jacobian;
}

GradientResults getGradient(const Eigen::VectorXd& dofvals,
                            const tesseract_collision::ContactResult& contact_result,
                            const Eigen::Vector3d& data,
                            const tesseract_kinematics::ForwardKinematics::ConstPtr& manip,
                            const tesseract_environment::AdjacencyMap::ConstPtr& adjacency_map,
                            const Eigen::Isometry3d& world_to_base)
{
  GradientResults results(data);
  //  results.error = std::max<double>((data[0] - contact_result.distance), 0.);
  //  results.error_with_buffer = std::max<double>((data[0] + data[1] - contact_result.distance), 0.);

  results.error = (data[0] - contact_result.distance);
  results.error_with_buffer = (data[0] + data[1] - contact_result.distance);
  for (std::size_t i = 0; i < 2; ++i)
  {
    tesseract_environment::AdjacencyMapPair::ConstPtr it = adjacency_map->getLinkMapping(contact_result.link_names[i]);
    if (it != nullptr)
      calcGradient(results, i, it, dofvals, contact_result, manip, world_to_base, false);
  }
  // DebugPrintInfo(res, results.gradients[0], results.gradients[1], dofvals, &res == &(dist_results.front()));

  return results;
}

GradientResults getGradient(const Eigen::VectorXd& dofvals0,
                            const Eigen::VectorXd& dofvals1,
                            const tesseract_collision::ContactResult& contact_result,
                            const Eigen::Vector3d& data,
                            const tesseract_kinematics::ForwardKinematics::ConstPtr& manip,
                            const tesseract_environment::AdjacencyMap::ConstPtr& adjacency_map,
                            const Eigen::Isometry3d& world_to_base,
                            bool isTimestep1)
{
  GradientResults results(data);
  //  results.error = std::max<double>((data[0] - contact_result.distance), 0.);
  //  results.error_with_buffer = std::max<double>((data[0] + data[1] - contact_result.distance), 0.);

  results.error = (data[0] - contact_result.distance);
  results.error_with_buffer = (data[0] + data[1] - contact_result.distance);

  Eigen::VectorXd dofvalst = Eigen::VectorXd::Zero(dofvals0.size());
  for (std::size_t i = 0; i < 2; ++i)
  {
    tesseract_environment::AdjacencyMapPair::ConstPtr it = adjacency_map->getLinkMapping(contact_result.link_names[i]);
    if (it != nullptr)
    {
      if (contact_result.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time0)
        dofvalst = dofvals0;
      else if (contact_result.cc_type[i] == tesseract_collision::ContinuousCollisionType::CCType_Time1)
        dofvalst = dofvals1;
      else
        dofvalst = dofvals0 + (dofvals1 - dofvals0) * contact_result.cc_time[i];

      calcGradient(results, i, it, dofvalst, contact_result, manip, world_to_base, isTimestep1);
    }
  }

  // DebugPrintInfo(res, results.gradients[0], results.gradients[1], dofvals, &res == &(dist_results.front()));

  return results;
}

void collisionsToDistances(const tesseract_collision::ContactResultVector& dist_results, std::vector<double>& dists)
{
  dists.clear();
  dists.reserve(dist_results.size());
  for (const auto& dist_result : dist_results)
    dists.push_back(dist_result.distance);
}

void debugPrintInfo(const tesseract_collision::ContactResult& res,
                    const Eigen::VectorXd& dist_grad_A,
                    const Eigen::VectorXd& dist_grad_B,
                    const Eigen::VectorXd& dof_vals,
                    bool header)
{
  if (header)
  {
    std::printf("\n");
    std::printf("DistanceResult| %30s | %30s | %6s | %6s, %6s, %6s | %6s, %6s, "
                "%6s | %6s, %6s, %6s | %6s, %6s, %6s | %6s, %6s, %6s | %10s %10s |",
                "LINK A",
                "LINK B",
                "DIST",
                "Nx",
                "Ny",
                "Nz",
                "PAx",
                "PAy",
                "PAz",
                "PBx",
                "PBy",
                "PBz",
                "LPAx",
                "LPAy",
                "LPAz",
                "LPBx",
                "LPBy",
                "LPBz",
                "CC TIME A",
                "CC TIME B");

    for (auto i = 0; i < dist_grad_A.size(); ++i)
    {
      if (i == dist_grad_A.size() - 1)
      {
        std::printf(" %6s |", ("dA" + std::to_string(i)).c_str());
      }
      else
      {
        std::printf(" %6s,", ("dA" + std::to_string(i)).c_str());
      }
    }

    for (auto i = 0; i < dist_grad_B.size(); ++i)
    {
      if (i == dist_grad_B.size() - 1)
      {
        std::printf(" %6s |", ("dB" + std::to_string(i)).c_str());
      }
      else
      {
        std::printf(" %6s,", ("dB" + std::to_string(i)).c_str());
      }
    }

    for (auto i = 0; i < dof_vals.size(); ++i)
    {
      if (i == dof_vals.size() - 1)
      {
        std::printf(" %6s |", ("J" + std::to_string(i)).c_str());
      }
      else
      {
        std::printf(" %6s,", ("J" + std::to_string(i)).c_str());
      }
    }

    std::printf("\n");
  }

  std::printf("DistanceResult| %30s | %30s | %6.3f | %6.3f, %6.3f, %6.3f | "
              "%6.3f, %6.3f, %6.3f | %6.3f, %6.3f, %6.3f | %6.3f, "
              "%6.3f, %6.3f | %6.3f, %6.3f, %6.3f | %10.3f %10.3f |",
              res.link_names[0].c_str(),
              res.link_names[1].c_str(),
              res.distance,
              res.normal(0),
              res.normal(1),
              res.normal(2),
              res.nearest_points[0](0),
              res.nearest_points[0](1),
              res.nearest_points[0](2),
              res.nearest_points[1](0),
              res.nearest_points[1](1),
              res.nearest_points[1](2),
              res.nearest_points_local[0](0),
              res.nearest_points_local[0](1),
              res.nearest_points_local[0](2),
              res.nearest_points_local[1](0),
              res.nearest_points_local[1](1),
              res.nearest_points_local[1](2),
              res.cc_time[0],
              res.cc_time[1]);

  for (auto i = 0; i < dist_grad_A.size(); ++i)
  {
    if (i == dist_grad_A.size() - 1)
    {
      std::printf(" %6.3f |", dist_grad_A(i));
    }
    else
    {
      std::printf(" %6.3f,", dist_grad_A(i));
    }
  }

  for (auto i = 0; i < dist_grad_B.size(); ++i)
  {
    if (i == dist_grad_B.size() - 1)
    {
      std::printf(" %6.3f |", dist_grad_B(i));
    }
    else
    {
      std::printf(" %6.3f,", dist_grad_B(i));
    }
  }

  for (auto i = 0; i < dof_vals.size(); ++i)
  {
    if (i == dof_vals.size() - 1)
    {
      std::printf(" %6.3f |", dof_vals(i));
    }
    else
    {
      std::printf(" %6.3f,", dof_vals(i));
    }
  }

  std::printf("\n");
}
}  // namespace trajopt
