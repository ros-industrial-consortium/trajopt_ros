/**
 * @file pagmo_problem_interface.h
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
#ifndef TRAJOPT_PAGMO_PAGMO_INTERFACE_H_
#define TRAJOPT_PAGMO_PAGMO_INTERFACE_H_

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/problem.h>
#include <pagmo/types.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_pagmo
{
/**
 * @brief Converts an ifopt::Problem into a Pagmo user defined problem
 *
 * Note that this struct is in a very specific format to be suitable to be converted to a Pagmo problem via type
 * erasure. Note that some functions are not implemented yet (e.g. hessians of get_nix) either because they don't apply
 * or are not yet supported.
 *
 * https://esa.github.io/pagmo2/docs/cpp/problem.html
 *
 */
struct PagmoProblemInterface
{
  PagmoProblemInterface() = default;

  void init(ifopt::Problem& nlp);

  // Number of equality constraints.
  std::vector<double>::size_type get_nec() const;
  /** @brief Number of inequality constraints. */
  std::vector<double>::size_type get_nic() const;

  /**
   * @brief This is the GetValue() of Pagmo.
   * @param decision_vec Input variables
   * @return This is a vector of {objective_function, eq_cnt_vec, ineq_cnt_vec}
   */
  std::vector<double> fitness(const std::vector<double>& decision_vec) const;

  /** @brief Returns the variable bounds */
  std::pair<std::vector<double>, std::vector<double>> get_bounds() const;

  bool has_gradient() const { return use_gradient_; }
  std::vector<double> gradient(const std::vector<double>& decision_vec) const;

  bool has_gradient_sparsity() const { return false; }

  /** @brief TODO: Add hessians to IFOPT */
  bool has_hessians() const { return false; }

  /** @brief Use the IFOPT gradient. Default = true */
  bool use_gradient_{ true };

private:
  std::vector<double> var_bounds_lower_;
  std::vector<double> var_bounds_upper_;
  Eigen::VectorXd cnt_bounds_lower_;
  Eigen::VectorXd cnt_bounds_upper_;

  ifopt::Problem* nlp_;
  std::vector<Eigen::Index> eq_cnt_idx_;
  std::vector<Eigen::Index> ineq_cnt_idx_;
};

}  // namespace trajopt_pagmo

#endif
