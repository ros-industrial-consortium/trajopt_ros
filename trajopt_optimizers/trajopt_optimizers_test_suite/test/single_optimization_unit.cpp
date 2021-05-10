#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>

#include <pagmo/algorithms/compass_search.hpp>
#include <pagmo/algorithms/gaco.hpp>
#include <pagmo/algorithms/ihs.hpp>
#include <pagmo/algorithms/nlopt.hpp>
#include <trajopt_pagmo/pagmo_problem_interface.h>
#include <trajopt_pagmo/pagmo_solver.h>

// Problem Header Files
#include <trajopt_optimizers_test_suite/single_optimization_unit.hpp>

/**
 * @brief Solves the basic optimization problem with the ipopt solver using exact jacobian approximation
 */
TEST_F(SingleOptimization, single_optimization_ipopt_jacobian_approximation)  // NOLINT
{
  ifopt::IpoptSolver solver;
  solver.SetOption("linear_solver", "mumps");
  solver.SetOption("jacobian_approximation", "exact");
  solver.SetOption("print_level", 5);

  runTests<ifopt::IpoptSolver>(solver, nlp_);
}

/**
 * @brief Solves the basic optimization problem with the ipopt solver using finite difference values
 */
TEST_F(SingleOptimization, single_optimization_ipopt_finite_difference_values)  // NOLINT
{
  ifopt::IpoptSolver solver;
  solver.SetOption("linear_solver", "mumps");
  solver.SetOption("jacobian_approximation", "finite-difference-values");
  solver.SetOption("print_level", 5);

  runTests<ifopt::IpoptSolver>(solver, nlp_);
}

/**
 * @brief Solves the basic optimization problem with the trajopt_sqp solver
 * NOTE: CURRENTLY BROKEN
 */
TEST_F(SingleOptimization, single_optimization_trajopt_sqp)  // NOLINT
{
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(DEBUG);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
  qp_solver->solver_.settings()->setMaxIteraction(8192);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  solver.verbose = DEBUG;
  runTests<trajopt_sqp::TrustRegionSQPSolver>(solver, nlp_);
}

/**
 * @brief Solves the basic optimization problem with Pagmo's Compass algorithm
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(SingleOptimization, single_optimization_trajopt_pagmo_compass) // NOLINT
// {
//   pagmo::compass_search uda(5000, 0.1, 1e-4, 0.5);
//   uda.set_verbosity(false);
//   pagmo::algorithm algo{ uda };
//   trajopt_pagmo::PagmoSolver solver{ algo };
//
//   runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_);
// }

/**
 * @brief Solves the basic optimization problem with Pagmo's Extended Ant Colony algorithm
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(SingleOptimization, single_optimization_trajopt_pagmo_gaco) // NOLINT
// {
//   pagmo::gaco uda(10);
//   uda.set_verbosity(false);
//   pagmo::algorithm algo{ uda };
//   trajopt_pagmo::PagmoSolver solver{ algo };
//   // Apparently Extended Ant Colony Optimization (GACO) works best with a lot of ants
//   solver.config_.population_size_ = 1000;
//
//   runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_);
// }

/**
 * @brief Solves the basic optimization problem with Pagmo's Improved Harmony Search algorithm
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(SingleOptimization, single_optimization_trajopt_pagmo_ihs) // NOLINT
// {
//   pagmo::ihs uda(100);
//   uda.set_verbosity(false);
//   pagmo::algorithm algo{ uda };
//   trajopt_pagmo::PagmoSolver solver{ algo };
//
//   runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_);
// }

/**
 * @brief Solves the basic optimization problem with Pagmo's implementation of NLopt's SQP solver
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(SingleOptimization, single_optimization_trajopt_pagmo_slsqp) // NOLINT
// {
//   pagmo::nlopt uda("slsqp");
//   uda.set_verbosity(false);
//   pagmo::algorithm algo{ uda };
//   // if(algo.has_set_seed()) { algo.set_seed(123u); };
//   trajopt_pagmo::PagmoSolver solver{ algo };
//   solver.config_.population_size_ = 100;
//   runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_);
// }
