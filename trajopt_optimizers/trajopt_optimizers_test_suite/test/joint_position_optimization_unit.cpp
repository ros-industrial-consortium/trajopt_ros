#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

// Solver Files
#include <pagmo/algorithms/compass_search.hpp>
#include <pagmo/algorithms/gaco.hpp>
#include <pagmo/algorithms/ihs.hpp>
#include <pagmo/algorithms/nlopt.hpp>
#include <trajopt_pagmo/pagmo_problem_interface.h>
#include <trajopt_pagmo/pagmo_solver.h>

#include <trajopt_optimizers_test_suite/joint_position_optimization_unit.hpp>

const bool VERBOSITY = false;

/**
 * @brief Applies a joint position constraint and solves the problem with IPOPT using exact jacobian approximation
 */
TEST_F(JointPositionOptimization, joint_position_optimization_ipopt_exact)  // NOLINT
{
  ifopt::IpoptSolver solver;
  solver.SetOption("derivative_test", "first-order");
  solver.SetOption("linear_solver", "mumps");
  solver.SetOption("jacobian_approximation", "exact");
  solver.SetOption("print_level", 5);
  runTests<ifopt::IpoptSolver>(solver, nlp_);
}

/**
 * @brief Applies a joint position constraint and solves the problem with IPOPT, using finite difference values
 */
TEST_F(JointPositionOptimization, joint_position_optimization_ipopt_fdv)  // NOLINT
{
  ifopt::IpoptSolver solver;
  solver.SetOption("derivative_test", "first-order");
  solver.SetOption("linear_solver", "mumps");
  solver.SetOption("jacobian_approximation", "finite-difference-values");
  solver.SetOption("print_level", 5);
  runTests<ifopt::IpoptSolver>(solver, nlp_);
}

/**
 * @brief Applies a joint position constraint and solves the problem with Pagmo's compass algorithm
 */
TEST_F(JointPositionOptimization, basic_optimization_trajopt_pagmo_compass)  // NOLINT
{
  pagmo::compass_search uda(5000, 0.1, 1e-6, 0.5);
  uda.set_verbosity(VERBOSITY);
  pagmo::algorithm algo{ uda };
  trajopt_pagmo::PagmoSolver solver{ algo };
  solver.config_.use_initial_vals_ = true;
  runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_);
}

/**
 * @brief Applies a joint position constraint and solves the problem with Pagmo's Extended Ant Colony Algorithm
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(JointPositionOptimization, joint_position_trajopt_pagmo_gaco) // NOLINT
// {
//   // pagmo::gaco uda(500,100, 0.5);
//   pagmo::gaco uda(100);
//   uda.set_verbosity(VERBOSITY);
//   pagmo::algorithm algo{ uda };
//   trajopt_pagmo::PagmoSolver solver{ algo };
//   // Apparently Extended Ant Colony Optimization (GACO) works best with a lot of ants
//   solver.config_.population_size_ = 100;
//   solver.config_.use_initial_vals_ = true;
//   runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_);
// }

/**
 * @brief Applies a joint position constraint and solves the problem with Pagmo's Improved Harmony Search
 * NOTE: CURRENTLY BROKEN
 */
// TEST_F(JointPositionOptimization, joint_position_trajopt_pagmo_ihs) // NOLINT
// {
//   pagmo::ihs uda(50);
//   uda.set_verbosity(VERBOSITY);
//   pagmo::algorithm algo{ uda };
//   trajopt_pagmo::PagmoSolver solver{ algo };
//   solver.config_.use_initial_vals_ = true;
//   runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_);
// }

/**
 * @brief Applies a joint position constraint and solves the problem with Pagmo's implementaiton of the NLopt's sqp
 * solver
 */
TEST_F(JointPositionOptimization, joint_position_trajopt_pagmo_slsqp)  // NOLINT
{
  pagmo::nlopt uda("slsqp");
  uda.set_verbosity(VERBOSITY);
  pagmo::algorithm algo{ uda };
  trajopt_pagmo::PagmoSolver solver{ algo };
  solver.config_.population_size_ = 100;
  solver.config_.use_initial_vals_ = true;
  runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_);
}

/**
 * @brief Applies a joint position constraint and solves the problem with Pagmo's implementaiton of the NLopt's COBYLA
 * solver
 */
TEST_F(JointPositionOptimization, joint_position_trajopt_pagmo_cobyla)  // NOLINT
{
  pagmo::nlopt uda("cobyla");
  uda.set_verbosity(VERBOSITY);
  pagmo::algorithm algo{ uda };
  trajopt_pagmo::PagmoSolver solver{ algo };
  solver.config_.population_size_ = 100;
  solver.config_.use_initial_vals_ = true;
  runTests<trajopt_pagmo::PagmoSolver>(solver, nlp_);
}
