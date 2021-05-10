/**
 * @file pagmo_solver.h
 * @brief Contains and implementation of Pagmo as an IFOPT Solver
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
#ifndef TRAJOPT_PAGMO_PAGMO_SOLVER_H_
#define TRAJOPT_PAGMO_PAGMO_SOLVER_H_

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/problem.h>
#include <ifopt/solver.h>
#include <pagmo/algorithm.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_pagmo
{
/**
 * @brief This is mostly a placeholder. Eventually it may contain information for setting up the island model
 */
struct PagmoSolverConfig
{
  using Ptr = std::shared_ptr<PagmoSolverConfig>;
  using ConstPtr = std::shared_ptr<const PagmoSolverConfig>;

  int verbose_{ 0 };
  bool use_gradient_{ true };
  bool use_initial_vals_{ false };
  std::size_t population_size_ = 24;
};

/**
 * @brief Implements Pagmo as an IFOPT Solver
 */
class PagmoSolver : public ifopt::Solver
{
public:
  using Ptr = std::shared_ptr<PagmoSolver>;
  using ConstPtr = std::shared_ptr<const PagmoSolver>;

  /**
   * @brief Constructor for the solver
   * @param pagmo_problem Preconfigured Pagmo UDA.
   *
   * Note that it must be compatible with the type of problem being solved
   * e.g. don't try to optimize constraints with a solver that is unconstrained only.
   *
   * If in doubt, you probably want a S-C or S-CU defined
   * https://esa.github.io/pagmo2/overview.html#list-of-algorithms
   */
  PagmoSolver(pagmo::algorithm& pagmo_algorithm);

  /**
   * @brief Solves and NLP using pagmo
   * @param nlp IFOPT problem to be solved
   */
  void Solve(ifopt::Problem& nlp) override;

  /**
   * @brief Debugs the problem and population of the solver
   * @param prob pagmo::problem of problem we are using
   * @param pop pagmo::population that we are operating on
   */
  void Debug(pagmo::problem& prob, pagmo::population& pop);

  /**
   * @brief Config used to set up Pagmo
   */
  PagmoSolverConfig config_;

private:
  pagmo::algorithm* pagmo_algorithm_;
  std::vector<double> initial_guess_;
};

}  // namespace trajopt_pagmo
#endif
