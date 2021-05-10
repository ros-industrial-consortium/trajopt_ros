#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

// Solvers
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <pagmo/algorithms/compass_search.hpp>
#include <pagmo/algorithms/gaco.hpp>
#include <pagmo/problems/hock_schittkowsky_71.hpp>
#include <pagmo/algorithms/ihs.hpp>
#include <pagmo/algorithms/nlopt.hpp>
#include <trajopt_pagmo/pagmo_problem_interface.h>
#include <trajopt_pagmo/pagmo_solver.h>

#include <trajopt_optimizers_test_suite/cart_position_optimization_unit.hpp>

const bool VERBOSITY = false;

/**
 * @brief Applies a cart position constraint and solves the problem with IPOPT using exact jacobian approximation
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(CartPositionOptimization, cart_position_optimization_ipopt_exact)  // NOLINT
// {
//   ifopt::IpoptSolver solver;
//   solver.SetOption("derivative_test", "first-order");
//   solver.SetOption("linear_solver", "mumps");
//   solver.SetOption("jacobian_approximation", "exact");
//   solver.SetOption("print_level", 5);
//   runTests<ifopt::IpoptSolver>(solver, nlp_, joint_target_);
// }

/**
 * @brief Applies a cart position constraint and solves the problem with IPOPT, using finite difference values
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(CartPositionOptimization, cart_position_optimization_ipopt_fdv)  // NOLINT
// {
//   ifopt::IpoptSolver solver;
//   solver.SetOption("derivative_test", "first-order");
//   solver.SetOption("linear_solver", "mumps");
//   solver.SetOption("jacobian_approximation", "finite-difference-values");
//   solver.SetOption("print_level", 5);
//   runTests<ifopt::IpoptSolver>(solver, nlp_, joint_target_);
// }

/**
 * @brief Solves the basic optimization problem with Pagmo's Compass algorithm
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(CartPositionOptimization, cart_optimization_trajopt_pagmo_compass) // NOLINT
// {
//   pagmo::compass_search uda(500, 0.1, 1e-7, 0.5);
//   uda.set_verbosity(VERBOSITY);
//   pagmo::algorithm algo{ uda };
//   trajopt_pagmo::PagmoSolver solver{ algo };
//   solver.config_.use_initial_vals_ = true;
//   runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_, joint_target_);
// }

/**
 * @brief Solves the basic optimization problem with Pagmo's Extended Ant Colony algorithm
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(CartPositionOptimization, cart_position_trajopt_pagmo_gaco) // NOLINT
// {
//   pagmo::gaco uda(100);
//   uda.set_verbosity(VERBOSITY);
//   pagmo::algorithm algo{ uda };
//   trajopt_pagmo::PagmoSolver solver{ algo };
//   // Apparently Extended Ant Colony Optimization (GACO) works best with a lot of ants
//   solver.config_.population_size_ = 100;
//   solver.config_.use_initial_vals_ = true;
//   runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_, joint_target_);
// }

/**
 * @brief Solves the basic optimization problem with Pagmo's Improved Harmony Search
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(CartPositionOptimization, cart_position_trajopt_pagmo_ihs) // NOLINT
// {
//   pagmo::ihs uda(50);
//   uda.set_verbosity(VERBOSITY);
//   pagmo::algorithm algo{ uda };
//   trajopt_pagmo::PagmoSolver solver{ algo };
//   solver.config_.use_initial_vals_ = true;
//   runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_, joint_target_);
// }

/**
 * @brief Solves the basic optimization problem with Pagmo's implementation of NLopt's SQP solver
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(CartPositionOptimization, cart_position_trajopt_pagmo_slsqp) // NOLINT
// {
//   pagmo::nlopt uda("slsqp");
//   uda.set_verbosity(VERBOSITY);
//   pagmo::algorithm algo{ uda };
//   trajopt_pagmo::PagmoSolver solver{ algo };
//   solver.config_.population_size_ = 500;
//   solver.config_.use_initial_vals_ = true;
//   runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_, joint_target_);
// }

/**
 * @brief Solves the basic optimization problem with Pagmo's implementation of NLopt's COBYLA solver
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(CartPositionOptimization, cart_position_trajopt_pagmo_cobyla) // NOLINT
// {
//   pagmo::nlopt uda("cobyla");
//   uda.set_verbosity(VERBOSITY);
//   pagmo::algorithm algo{ uda };
//   trajopt_pagmo::PagmoSolver solver{ algo };
//   solver.config_.population_size_ = 100;
//   solver.config_.use_initial_vals_ = true;
//   runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_, joint_target_);
// }
