/**
 * @file pagmo_solver.cpp
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
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <iostream>
#include <console_bridge/console.h>

#include <pagmo/algorithm.hpp>
#include <pagmo/population.hpp>
#include <pagmo/problem.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_pagmo/pagmo_solver.h>
#include <trajopt_pagmo/pagmo_problem_interface.h>

namespace trajopt_pagmo
{
PagmoSolver::PagmoSolver(pagmo::algorithm& pagmo_algorithm) { pagmo_algorithm_ = &pagmo_algorithm; };

void PagmoSolver::Solve(ifopt::Problem& nlp)
{
  // 1 - Instantiate a pagmo problem constructing it from a UDP
  // (user defined problem).
  PagmoProblemInterface pagmo_interface{};
  pagmo_interface.init(nlp);
  pagmo_interface.use_gradient_ = true;  // config_.use_gradient_;
  pagmo::problem pagmo_problem{ pagmo_interface };

  // 2 - Instantiate a population
  pagmo::population pop{ pagmo_problem, config_.population_size_, 0u };

  // 3 - Set Initial Values
  Eigen::VectorXd var_vals = nlp.GetOptVariables()->GetValues();
  std::vector<double> init_vals(var_vals.data(), var_vals.data() + var_vals.rows() * var_vals.cols());
  if (config_.use_initial_vals_)
  {
    for (size_t i = 0; i < config_.population_size_; i++)
    {
      pop.set_x(i, init_vals);
    }
  }

  // 4 - Evolve the population
  pop = pagmo_algorithm_->evolve(pop);

  // 5 - Set the results
  pagmo::vector_double results = pop.champion_x();
  nlp.SetVariables(results.data());
}

inline void PagmoSolver::Debug(pagmo::problem& prob, pagmo::population& pop)
{
  std::cout << prob << '\n';
  std::cout << pop << '\n';
  std::cout << std::endl;
}

}  // namespace trajopt_pagmo
