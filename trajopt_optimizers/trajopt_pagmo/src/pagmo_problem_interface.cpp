/**
 * @file pagmo_problem_interface.cpp
 * @brief Converts an ifopt::Problem into a Pagmo user defined problem
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

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <iostream>
#include <console_bridge/console.h>

#include <trajopt_pagmo/pagmo_problem_interface.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_pagmo
{
void PagmoProblemInterface::init(ifopt::Problem& nlp)
{
  nlp_ = &nlp;

  ////////////////////////////////////////////////////////
  // Get NLP bounds and detect constraint type
  ////////////////////////////////////////////////////////
  cnt_bounds_lower_.resize(nlp.GetNumberOfConstraints());
  cnt_bounds_upper_.resize(nlp.GetNumberOfConstraints());
  // Convert constraint bounds to VectorXd
  std::vector<ifopt::Bounds> cnt_bounds = nlp_->GetBoundsOnConstraints();
  for (Eigen::Index i = 0; i < nlp.GetNumberOfConstraints(); i++)
  {
    cnt_bounds_lower_[i] = cnt_bounds[static_cast<std::size_t>(i)].lower_;
    cnt_bounds_upper_[i] = cnt_bounds[static_cast<std::size_t>(i)].upper_;
  }

  // Detect constraint type and store indices
  Eigen::VectorXd bounds_diff = cnt_bounds_upper_ - cnt_bounds_lower_;
  eq_cnt_idx_.clear();
  ineq_cnt_idx_.clear();
  for (Eigen::Index i = 0; i < bounds_diff.size(); i++)
  {
    if (std::abs(bounds_diff[i]) < 1e-3)
      eq_cnt_idx_.push_back(i);
    else
      ineq_cnt_idx_.push_back(i);
  }

  ////////////////////////////////////////////////////////
  // Get Variable bounds
  ////////////////////////////////////////////////////////
  var_bounds_lower_.resize(static_cast<std::size_t>(nlp.GetNumberOfOptimizationVariables()));
  var_bounds_upper_.resize(static_cast<std::size_t>(nlp.GetNumberOfOptimizationVariables()));
  // Convert constraint bounds to VectorXd
  std::vector<ifopt::Bounds> var_bounds = nlp_->GetBoundsOnOptimizationVariables();
  for (std::size_t i = 0; i < static_cast<std::size_t>(nlp.GetNumberOfConstraints()); i++)
  {
    var_bounds_lower_[i] = var_bounds[i].lower_;
    var_bounds_upper_[i] = var_bounds[i].upper_;
  }
}

std::vector<double>::size_type PagmoProblemInterface::get_nec() const { return eq_cnt_idx_.size(); }

std::vector<double>::size_type PagmoProblemInterface::get_nic() const { return ineq_cnt_idx_.size(); }

std::vector<double> PagmoProblemInterface::fitness(const std::vector<double>& decision_vec) const
{
  std::vector<double> f(1 + eq_cnt_idx_.size() + ineq_cnt_idx_.size(), 0.);

  // Evaluate objective function
  f[0] = nlp_->EvaluateCostFunction(decision_vec.data());

  // Evaluate constraints - TODO: May be more efficient way of doing this.
  Eigen::VectorXd constraints = nlp_->EvaluateConstraints(decision_vec.data());
  std::size_t j = 1;
  for (const Eigen::Index& i : eq_cnt_idx_)
  {
    // cnt = bounds => cnt - bounds = 0
    f[j] = constraints[i] - cnt_bounds_lower_[i];
    j++;
  }
  for (const Eigen::Index& i : ineq_cnt_idx_)
  {
    // Calculate distance from bounds. Positive is outside the boundary
    double dist_lower = cnt_bounds_lower_[i] - constraints[i];
    double dist_upper = -(cnt_bounds_upper_[i] - constraints[i]);

    // Take the largest violation (or distance to closest boundary)
    f[j] = fmax(dist_lower, dist_upper);
    j++;
  }
  return f;
}

std::pair<std::vector<double>, std::vector<double>> PagmoProblemInterface::get_bounds() const
{
  return { var_bounds_lower_, var_bounds_upper_ };
}

std::vector<double> PagmoProblemInterface::gradient(const std::vector<double>& decision_vec) const
{
  // Add Cost Gradients
  Eigen::VectorXd cost_grad = nlp_->EvaluateCostFunctionGradient(decision_vec.data());
  std::vector<double> grad_vec(cost_grad.data(), cost_grad.data() + cost_grad.rows() * cost_grad.cols());

  // Add Constraint Gradients. Goes row by row in the jacobian and turns them into an nx1 vector
  ifopt::Problem::Jacobian prob_jac = nlp_->GetJacobianOfConstraints();
  for (int i = 0; i < nlp_->GetNumberOfConstraints(); i++)
  {
    Eigen::VectorXd jac_grad_vec = prob_jac.row(i).transpose();
    std::vector<double> cg(jac_grad_vec.data(), jac_grad_vec.data() + jac_grad_vec.rows() * jac_grad_vec.cols());
    grad_vec.insert(grad_vec.end(), cg.begin(), cg.end());
  }

  return grad_vec;
};

}  // namespace trajopt_pagmo
