/**
 * @file trust_region_sqp_solver.cpp
 * @brief Contains the main trust region SQP solver. While it is based on the paper below, it has been completely
 * rewritten from trajopt_sco
 *
 * Schulman, J., Ho, J., Lee, A. X., Awwal, I., Bradlow, H., & Abbeel, P. (2013, June). Finding Locally Optimal,
 * Collision-Free Trajectories with Sequential Convex Optimization. In Robotics: science and systems (Vol. 9, No. 1, pp.
 * 1-10).
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
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <iostream>
#include <console_bridge/console.h>

namespace trajopt_sqp
{
const bool SUPER_DEBUG_MODE = true;

TrustRegionSQPSolver::TrustRegionSQPSolver(QPSolver::Ptr qp_solver) : qp_solver(std::move(qp_solver)) {}

bool TrustRegionSQPSolver::init(QPProblem::Ptr qp_prob)
{
  qp_problem = qp_prob;

  // Initialize optimization parameters
  results_ = SQPResults(qp_problem->getNumNLPVars(), qp_problem->getNumNLPConstraints(), qp_problem->getNumNLPCosts());
  results_.best_var_vals = qp_problem->getVariableValues();

  // Evaluate exact constraint violations (expensive)
  results_.best_costs = qp_problem->getExactCosts();

  // Evaluate exact constraint violations (expensive)
  results_.best_constraint_violations = qp_problem->getExactConstraintViolations();

  // Calculate exact NLP merits (expensive) - TODO: Look into caching for qp_solver->Convexify()
  results_.best_exact_merit = qp_problem->evaluateTotalExactCost(results_.best_var_vals) +
                              results_.best_constraint_violations.dot(results_.merit_error_coeffs);

  setBoxSize(params.initial_trust_box_size);
  return true;
}

void TrustRegionSQPSolver::setBoxSize(double box_size)
{
  results_.box_size = Eigen::VectorXd::Constant(qp_problem->getNumNLPVars(), box_size);
  qp_problem->setBoxSize(results_.box_size);
}

void TrustRegionSQPSolver::registerCallback(const SQPCallback::Ptr& callback) { callbacks_.push_back(callback); }

const SQPStatus& TrustRegionSQPSolver::getStatus() { return status_; }

const SQPResults& TrustRegionSQPSolver::getResults() { return results_; }

void TrustRegionSQPSolver::solve(QPProblem::Ptr qp_problem)
{
  status_ = SQPStatus::RUNNING;

  // Initialize solver
  init(qp_problem);

  // Penalty Iteration Loop
  for (int penalty_iteration = 0; penalty_iteration < params.max_merit_coeff_increases; penalty_iteration++)
  {
    results_.penalty_iteration = penalty_iteration;
    results_.convexify_iteration = 0;

    // Convexification loop
    for (int convex_iteration = 0; convex_iteration < 100; convex_iteration++)
    {
      if (stepSQPSolver())
        break;

      if (results_.overall_iteration >= params.max_iterations)
      {
        CONSOLE_BRIDGE_logInform("Iteration limit");
        status_ = SQPStatus::ITERATION_LIMIT;
        break;
      }
    }

    // Check if constraints are satisfied
    if (verifySQPSolverConvergence())
      break;

    // ---------------------------
    // Constraints are not satisfied!
    // Penalty Adjustment
    // ---------------------------
    adjustPenalty();
  }  // Penalty adjustment loop

  // Final Cleanup
  if (SUPER_DEBUG_MODE)
    results_.print();

  qp_problem->setVariables(results_.best_var_vals.data());
}

bool TrustRegionSQPSolver::verifySQPSolverConvergence()
{
  // Check if constrainsts are satisfied
  if (results_.new_constraint_violations.size() == 0)
  {
    CONSOLE_BRIDGE_logInform("Optimization has converged and there are no constraints");
    return true;
  }

  if (results_.new_constraint_violations.maxCoeff() < params.cnt_tolerance)
  {
    CONSOLE_BRIDGE_logInform("woo-hoo! constraints are satisfied (to tolerance %.2e)", params.cnt_tolerance);
    return true;
  }

  return false;
}

void TrustRegionSQPSolver::adjustPenalty()
{
  if (params.inflate_constraints_individually)
  {
    assert(results_.new_constraint_violations.size() == results_.merit_error_coeffs.size());
    for (Eigen::Index idx = 0; idx < results_.new_constraint_violations.size(); idx++)
    {
      if (results_.new_constraint_violations[idx] > params.cnt_tolerance)
      {
        CONSOLE_BRIDGE_logInform("Not all constraints are satisfied. Increasing constraint penalties for %d", idx);
        results_.merit_error_coeffs[idx] *= params.merit_coeff_increase_ratio;
      }
    }
  }
  else
  {
    CONSOLE_BRIDGE_logInform("Not all constraints are satisfied. Increasing constraint penalties uniformly");
    results_.merit_error_coeffs *= params.merit_coeff_increase_ratio;
  }
  results_.box_size = Eigen::VectorXd::Ones(results_.box_size.size()) *
                      fmax(results_.box_size[0], params.min_trust_box_size / params.trust_shrink_ratio * 1.5);
}

bool TrustRegionSQPSolver::stepSQPSolver()
{
  results_.convexify_iteration++;
  qp_problem->convexify();

  // TODO: Look into not clearing and reinitializing the workspace each iteration. It should be as simple as
  // removing this
  qp_solver->clear();

  // Convexify the costs and constraints around their current values
  qp_solver->init(qp_problem->getNumQPVars(), qp_problem->getNumQPConstraints());
  qp_solver->updateHessianMatrix(qp_problem->getHessian());
  qp_solver->updateGradient(qp_problem->getGradient());
  qp_solver->updateLinearConstraintsMatrix(qp_problem->getConstraintMatrix());
  qp_solver->updateBounds(qp_problem->getBoundsLower(), qp_problem->getBoundsUpper());

  // Trust region loop
  runTrustRegionLoop();

  // Check if the NLP has converged
  if (status_ == SQPStatus::NLP_CONVERGED)
    return true;

  if (results_.box_size.maxCoeff() < params.min_trust_box_size)
  {
    CONSOLE_BRIDGE_logInform("Converged because trust region is tiny");
    status_ = SQPStatus::NLP_CONVERGED;
    return true;
  }
  return false;
}

void TrustRegionSQPSolver::runTrustRegionLoop()
{
  for (int trust_region_iteration = 0; trust_region_iteration < params.max_trust_region_expansions;
       trust_region_iteration++)
  {
    if (SUPER_DEBUG_MODE)
      qp_problem->print();

    results_.overall_iteration++;
    results_.trust_region_iteration = trust_region_iteration + 1;

    // Solve the current QP problem
    status_ = solveQPProblem();
    if (status_ != SQPStatus::RUNNING)
    {
      qp_problem->setVariables(results_.best_var_vals.data());
      return;
    }

    // Check if the entire NLP Converged
    if (results_.approx_merit_improve < -1e-5)
    {
      CONSOLE_BRIDGE_logError("Approximate merit function got worse (%.3e). (convexification is probably wrong to "
                              "zeroth order)",
                              results_.approx_merit_improve);
    }

    if (results_.approx_merit_improve < params.min_approx_improve)
    {
      CONSOLE_BRIDGE_logInform("Converged because improvement was small (%.3e < %.3e)",
                               results_.approx_merit_improve,
                               params.min_approx_improve);
      status_ = SQPStatus::NLP_CONVERGED;
      return;
    }

    if (results_.approx_merit_improve / results_.best_exact_merit < params.min_approx_improve_frac)
    {
      CONSOLE_BRIDGE_logInform("Converged because improvement ratio was small (%.3e < %.3e)",
                               results_.approx_merit_improve / results_.best_exact_merit,
                               params.min_approx_improve_frac);
      status_ = SQPStatus::NLP_CONVERGED;
      return;
    }

    // Check if the bounding trust region needs to be shrunk
    // This happens if the exact solution got worse or if the QP approximation deviates from the exact by too much
    if (results_.exact_merit_improve < 0 || results_.merit_improve_ratio < params.improve_ratio_threshold)
    {
      qp_problem->scaleBoxSize(params.trust_shrink_ratio);
      qp_solver->updateBounds(qp_problem->getBoundsLower(), qp_problem->getBoundsUpper());
      results_.box_size = qp_problem->getBoxSize();

      CONSOLE_BRIDGE_logInform("Shrunk trust region. new box size: %.4f", results_.box_size[0]);
    }
    else
    {
      results_.best_var_vals = results_.new_var_vals;

      results_.best_exact_merit = results_.new_exact_merit;
      results_.best_constraint_violations = results_.new_constraint_violations;
      results_.best_costs = results_.new_costs;

      results_.best_approx_merit = results_.new_approx_merit;
      results_.best_approx_constraint_violations = results_.new_approx_constraint_violations;
      results_.best_approx_costs = results_.new_approx_costs;

      if (SUPER_DEBUG_MODE)
        results_.print();

      qp_problem->setVariables(results_.best_var_vals.data());

      qp_problem->scaleBoxSize(params.trust_expand_ratio);
      results_.box_size = qp_problem->getBoxSize();
      CONSOLE_BRIDGE_logInform("Expanded trust region. new box size: %.4f", results_.box_size[0]);
      return;
    }
  }  // Trust region loop
}

SQPStatus TrustRegionSQPSolver::solveQPProblem()
{
  // Solve the QP
  bool succeed = qp_solver->solve();

  if (succeed)
  {
    results_.new_var_vals = qp_solver->getSolution();

    // Calculate approximate QP merits (cheap)
    qp_problem->setVariables(results_.new_var_vals.data());

    // Evaluate convexified constraint violations (expensive)
    results_.new_approx_constraint_violations = qp_problem->evaluateConvexConstraintViolations(results_.new_var_vals);

    // Evaluate convexified costs (expensive)
    results_.new_approx_costs = qp_problem->evaluateConvexCosts(results_.new_var_vals);

    // Convexified merit
    results_.new_approx_merit =
        results_.new_approx_costs.sum() + results_.new_approx_constraint_violations.dot(results_.merit_error_coeffs);

    results_.approx_merit_improve = results_.best_exact_merit - results_.new_approx_merit;

    // Evaluate exact costs (expensive)
    results_.new_costs = qp_problem->evaluateExactCosts(results_.new_var_vals);

    // Evaluate exact constraint violations (expensive)
    results_.new_constraint_violations = qp_problem->evaluateExactConstraintViolations(results_.new_var_vals);

    // Calculate exact NLP merits (expensive) - TODO: Look into caching for qp_solver->Convexify()
    results_.new_exact_merit =
        results_.new_costs.sum() + results_.new_constraint_violations.dot(results_.merit_error_coeffs);
    results_.exact_merit_improve = results_.best_exact_merit - results_.new_exact_merit;
    results_.merit_improve_ratio = results_.exact_merit_improve / results_.approx_merit_improve;

    // The variable are changed to the new values to calculated data but must be set
    // to best var vals because the new values may not improve the merit which is determined later.
    qp_problem->setVariables(results_.best_var_vals.data());

    // Print debugging info
    if (verbose)
      printStepInfo();

    // Call callbacks
    succeed &= callCallbacks();
  }
  else
  {
    CONSOLE_BRIDGE_logError("Solver Failure");
    return SQPStatus::QP_SOLVER_ERROR;
  }

  // Check if any callbacks returned false
  if (!succeed)
  {
    return SQPStatus::CALLBACK_STOPPED;
  }

  return SQPStatus::RUNNING;
}

bool TrustRegionSQPSolver::callCallbacks()
{
  bool success = true;
  //  for (const auto& callback : callbacks_)
  //    success &= callback->execute(*nlp_, results_);
  return success;
}

void TrustRegionSQPSolver::printStepInfo() const
{
  // Print Header
  std::printf("\n| %s |\n", std::string(88, '=').c_str());
  std::printf("| %s %s %s |\n", std::string(36, ' ').c_str(), "ROS Industrial", std::string(36, ' ').c_str());
  std::printf("| %s %s %s |\n", std::string(32, ' ').c_str(), "TrajOpt Motion Planning", std::string(31, ' ').c_str());
  std::printf("| %s |\n", std::string(88, '=').c_str());
  std::printf("| %s %s (Box Size: %-3.9f) %s |\n",
              std::string(26, ' ').c_str(),
              "Iteration",
              results_.box_size(0),
              std::string(27, ' ').c_str());
  std::printf("| %s |\n", std::string(88, '-').c_str());
  std::printf("| %14s: %-4d | %14s: %-4d | %15s: %-3d | %14s: %-3d"
              "d |\n",
              "Overall",
              results_.overall_iteration,
              "Convexify",
              results_.convexify_iteration,
              "Trust Region",
              results_.trust_region_iteration,
              "Penalty",
              results_.penalty_iteration);
  std::printf("| %s |\n", std::string(88, '=').c_str());

  // Print Cost and Constraint Data
  std::printf("| %10s | %10s | %10s | %10s | %10s | %10s | %10s |\n",
              "merit",
              "oldexact",
              "new_exact",
              "new_approx",
              "dapprox",
              "dexact",
              "ratio");

  // Individual Costs
  std::printf("| %s | INDIVIDUAL COSTS\n", std::string(88, '-').c_str());
  // Loop over costs
  const std::vector<std::string>& cost_names = qp_problem->getNLPCostNames();
  for (Eigen::Index cost_number = 0; cost_number < static_cast<Eigen::Index>(cost_names.size()); ++cost_number)
  {
    double approx_improve = results_.best_costs[cost_number] - results_.new_approx_costs[cost_number];
    double exact_improve = results_.best_costs[cost_number] - results_.new_costs[cost_number];
    if (fabs(approx_improve) > 1e-8)
      std::printf("| %10s | %10.3e | %10.3e | %10.3e | %10.3e | %10.3e | %10.3e | %-15s \n",
                  "----------",
                  results_.best_costs[cost_number],
                  results_.new_costs[cost_number],
                  results_.new_approx_costs[cost_number],
                  approx_improve,
                  exact_improve,
                  exact_improve / approx_improve,
                  cost_names[static_cast<std::size_t>(cost_number)].c_str());
    else
      std::printf("| %10s | %10.3e | %10.3e | %10.3e | %10.3e | %10.3e | %10s | %-15s\n",
                  "----------",
                  results_.best_costs[cost_number],
                  results_.new_costs[cost_number],
                  results_.new_approx_costs[cost_number],
                  approx_improve,
                  exact_improve,
                  "  ------  ",
                  cost_names[static_cast<std::size_t>(cost_number)].c_str());
  }

  // Sum Cost
  std::printf("| %s | SUM COSTS\n", std::string(88, '-').c_str());
  std::printf("| %10s | %10.3e | %10.3e | %10.3e | %10s | %10s | %10s |\n",
              "----------",
              results_.best_costs.sum(),
              results_.new_costs.sum(),
              results_.new_approx_costs.sum(),
              "----------",
              "----------",
              "----------");

  // Individual Constraints
  // If we want to print the names we will have to add a getConstraints function to IFOPT
  if (results_.new_constraint_violations.size() != 0)
  {
    std::printf("| %s | CONSTRAINTS\n", std::string(88, '-').c_str());
    const std::vector<std::string>& constraint_names = qp_problem->getNLPConstraintNames();
    // Loop over constraints
    for (Eigen::Index cnt_number = 0; cnt_number < static_cast<Eigen::Index>(constraint_names.size()); ++cnt_number)
    {
      double approx_improve =
          results_.best_constraint_violations[cnt_number] - results_.new_approx_constraint_violations[cnt_number];
      double exact_improve =
          results_.best_constraint_violations[cnt_number] - results_.new_constraint_violations[cnt_number];
      if (fabs(approx_improve) > 1e-8)
        std::printf("| %10.3e | %10.3e | %10.3e | %10.3e | %10.3e | %10.3e | %10.3e | %-15s\n",
                    results_.merit_error_coeffs[cnt_number],
                    results_.merit_error_coeffs[cnt_number] * results_.best_constraint_violations[cnt_number],
                    results_.merit_error_coeffs[cnt_number] * results_.new_constraint_violations[cnt_number],
                    results_.merit_error_coeffs[cnt_number] * results_.new_approx_constraint_violations[cnt_number],
                    results_.merit_error_coeffs[cnt_number] * approx_improve,
                    results_.merit_error_coeffs[cnt_number] * exact_improve,
                    exact_improve / approx_improve,
                    constraint_names[static_cast<std::size_t>(cnt_number)].c_str());
      else
        std::printf("| %10.3e | %10.3e | %10.3e | %10.3e | %10.3e | %10.3e | %10s | %-15s \n",
                    results_.merit_error_coeffs[cnt_number],
                    results_.merit_error_coeffs[cnt_number] * results_.best_constraint_violations[cnt_number],
                    results_.merit_error_coeffs[cnt_number] * results_.new_constraint_violations[cnt_number],
                    results_.merit_error_coeffs[cnt_number] * results_.new_approx_constraint_violations[cnt_number],
                    results_.merit_error_coeffs[cnt_number] * approx_improve,
                    results_.merit_error_coeffs[cnt_number] * exact_improve,
                    "  ------  ",
                    constraint_names[static_cast<std::size_t>(cnt_number)].c_str());
    }
  }

  // Constraint
  std::printf("| %s | SUM CONSTRAINTS\n", std::string(88, '-').c_str());
  std::printf("| %10s | %10.3e | %10.3e | %10.3e | %10s | %10s | %10s |\n",
              "----------",
              results_.best_constraint_violations.sum(),
              results_.new_constraint_violations.sum(),
              results_.new_approx_constraint_violations.sum(),
              "----------",
              "----------",
              "----------");

  // Total
  std::printf("| %s |\n", std::string(88, '=').c_str());
  std::printf("| %10s | %10.3e | %10.3e | %10s | %10.3e | %10.3e | %10.3e | TOTAL (SUM COSTS + SUM CONSTRAINTS)\n",
              "----------",
              results_.best_exact_merit,
              results_.new_exact_merit,
              "----------",
              results_.approx_merit_improve,
              results_.exact_merit_improve,
              results_.merit_improve_ratio);
  std::printf("| %s |\n", std::string(88, '=').c_str());
}

}  // namespace trajopt_sqp
