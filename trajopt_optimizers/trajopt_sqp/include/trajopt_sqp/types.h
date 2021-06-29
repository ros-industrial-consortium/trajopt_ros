/**
 * @file types.h
 * @brief Contains types for the trust region sqp solver
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
#ifndef TRAJOPT_SQP_TYPES_H_
#define TRAJOPT_SQP_TYPES_H_

#include <Eigen/Eigen>
#include <iostream>

namespace trajopt_sqp
{
using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;
using Hessian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

enum class ConstraintType
{
  EQ,
  INEQ
};

enum class CostPenaltyType
{
  SQUARED,
  ABSOLUTE
};

/**
 * @brief This struct defines parameters for the SQP optimization. The optimization should not change this struct
 */
struct SQPParameters
{
  double improve_ratio_threshold = 0.25;
  /** @brief NLP converges if trust region is smaller than this */
  double min_trust_box_size = 1e-4;
  /** @brief NLP converges if approx_merit_improves is smaller than this */
  double min_approx_improve = 1e-4;
  /** @brief NLP converges if approx_merit_improve / best_exact_merit < min_approx_improve_frac */
  double min_approx_improve_frac = static_cast<double>(-INFINITY);
  /** @brief Max number of QP calls allowed */
  int max_iterations = 50;

  /** @brief Trust region is scaled by this when it is shrunk */
  double trust_shrink_ratio = 0.1;
  /** @brief Trust region is expanded by this when it is expanded */
  double trust_expand_ratio = 1.5;
  /** @brief Max number of times the trust region will be expanded */
  int max_trust_region_expansions = 10;

  /** @brief Any constraint under this value is not considered a violation */
  double cnt_tolerance = 1e-4;
  /** @brief Max number of times the constraints will be inflated */
  double max_merit_coeff_increases = 5;
  /** @brief Constraints are scaled by this amount when inflated */
  double merit_coeff_increase_ratio = 10;
  /** @brief Unused */
  double max_time = static_cast<double>(INFINITY);
  /** @brief If true, only the constraints that are violated will be inflated */
  bool inflate_constraints_individually = true;
  /** @brief Initial size of the trust region */
  double initial_trust_box_size = 1e-1;
  /** @brief Unused */
  double log_results = false;
  /** @brief Unused */
  std::string log_dir = "/tmp";
};

/** @brief This struct contains information and results for the SQP problem */
struct SQPResults
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SQPResults(Eigen::Index num_vars, Eigen::Index num_cnts, Eigen::Index num_costs)
  {
    best_constraint_violations = Eigen::VectorXd::Zero(num_cnts);
    new_constraint_violations = Eigen::VectorXd::Zero(num_cnts);
    best_approx_constraint_violations = Eigen::VectorXd::Zero(num_cnts);
    new_approx_constraint_violations = Eigen::VectorXd::Zero(num_cnts);

    best_costs = Eigen::VectorXd::Zero(num_costs);
    new_costs = Eigen::VectorXd::Zero(num_costs);
    best_approx_costs = Eigen::VectorXd::Zero(num_costs);
    new_approx_costs = Eigen::VectorXd::Zero(num_costs);

    best_var_vals = Eigen::VectorXd::Zero(num_vars);
    new_var_vals = Eigen::VectorXd::Zero(num_vars);
    box_size = Eigen::VectorXd::Ones(num_vars);
    merit_error_coeffs = Eigen::VectorXd::Constant(num_cnts, 10);
  }
  SQPResults() = default;
  /** @brief The lowest cost ever achieved */
  double best_exact_merit{ std::numeric_limits<double>::max() };
  /** @brief The cost achieved this iteration */
  double new_exact_merit{ std::numeric_limits<double>::max() };
  /** @brief The lowest convexified cost ever achieved */
  double best_approx_merit{ std::numeric_limits<double>::max() };
  /** @brief The convexified cost achieved this iteration */
  double new_approx_merit{ std::numeric_limits<double>::max() };

  /** @brief Variable values associated with best_exact_merit */
  Eigen::VectorXd best_var_vals;
  /** @brief Variable values associated with this iteration */
  Eigen::VectorXd new_var_vals;

  /** @brief Amount the convexified cost improved over the best this iteration */
  double approx_merit_improve{ 0 };
  /** @brief Amount the exact cost improved over the best this iteration */
  double exact_merit_improve{ 0 };
  /** @brief The amount the cost improved as a ratio of the total cost */
  double merit_improve_ratio{ 0 };

  /** @brief Vector defing the box size. The box is var_vals +/- box_size */
  Eigen::VectorXd box_size;
  /** @brief Coefficients used to weight the constraint violations */
  Eigen::VectorXd merit_error_coeffs;

  /** @brief Vector of the constraint violations. Positive is a violation */
  Eigen::VectorXd best_constraint_violations;
  /** @brief Vector of the constraint violations. Positive is a violation */
  Eigen::VectorXd new_constraint_violations;

  /** @brief Vector of the convexified constraint violations. Positive is a violation */
  Eigen::VectorXd best_approx_constraint_violations;
  /** @brief Vector of the convexified constraint violations. Positive is a violation */
  Eigen::VectorXd new_approx_constraint_violations;

  /** @brief Vector of the constraint violations. Positive is a violation */
  Eigen::VectorXd best_costs;
  /** @brief Vector of the constraint violations. Positive is a violation */
  Eigen::VectorXd new_costs;

  /** @brief Vector of the convexified costs.*/
  Eigen::VectorXd best_approx_costs;
  /** @brief Vector of the convexified costs.*/
  Eigen::VectorXd new_approx_costs;

  int penalty_iteration{ 0 };
  int convexify_iteration{ 0 };
  int trust_region_iteration{ 0 };
  int overall_iteration{ 0 };

  void print() const
  {
    Eigen::IOFormat format(3);
    std::cout << "-------------- SQPResults::print() --------------" << std::endl;
    std::cout << "best_exact_merit: " << best_exact_merit << std::endl;
    std::cout << "new_exact_merit: " << new_exact_merit << std::endl;
    std::cout << "best_approx_merit: " << best_approx_merit << std::endl;
    std::cout << "new_approx_merit: " << new_approx_merit << std::endl;

    std::cout << "best_var_vals: " << best_var_vals.transpose().format(format) << std::endl;
    std::cout << "new_var_vals: " << new_var_vals.transpose().format(format) << std::endl;

    std::cout << "approx_merit_improve: " << approx_merit_improve << std::endl;
    std::cout << "exact_merit_improve: " << exact_merit_improve << std::endl;
    std::cout << "merit_improve_ratio: " << merit_improve_ratio << std::endl;

    std::cout << "box_size: " << box_size.transpose().format(format) << std::endl;
    std::cout << "merit_error_coeffs: " << merit_error_coeffs.transpose().format(format) << std::endl;

    std::cout << "best_constraint_violations: " << best_constraint_violations.transpose().format(format) << std::endl;
    std::cout << "new_constraint_violations: " << new_constraint_violations.transpose().format(format) << std::endl;
    std::cout << "best_approx_constraint_violations: " << best_approx_constraint_violations.transpose().format(format)
              << std::endl;
    std::cout << "new_approx_constraint_violations: " << new_approx_constraint_violations.transpose().format(format)
              << std::endl;

    std::cout << "best_costs: " << best_costs.transpose().format(format) << std::endl;
    std::cout << "new_costs: " << new_costs.transpose().format(format) << std::endl;
    std::cout << "best_approx_costs: " << best_approx_costs.transpose().format(format) << std::endl;
    std::cout << "new_approx_costs: " << new_approx_costs.transpose().format(format) << std::endl;

    std::cout << "penalty_iteration: " << penalty_iteration << std::endl;
    std::cout << "convexify_iteration: " << convexify_iteration << std::endl;
    std::cout << "trust_region_iteration: " << trust_region_iteration << std::endl;
    std::cout << "overall_iteration: " << overall_iteration << std::endl;
  }
};

/**
 * @brief Status codes for the SQP Optimization
 */
enum class SQPStatus
{
  RUNNING,         /**< Optimization is currently running */
  NLP_CONVERGED,   /**< NLP Successfully converged */
  ITERATION_LIMIT, /**< SQP Optimization reached iteration limit */
  QP_SOLVER_ERROR, /**< QP Solver failed */
  CALLBACK_STOPPED /**< Optimization stopped because callback returned false */
};

}  // namespace trajopt_sqp

#endif
