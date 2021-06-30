/**
 * @file trust_region_sqp_solver.h
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
#ifndef TRAJOPT_SQP_INCLUDE_SIMPLE_SQP_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_SIMPLE_SQP_SOLVER_H_

#include <trajopt_sqp/qp_problem.h>
#include <trajopt_sqp/qp_solver.h>
#include <trajopt_sqp/sqp_callback.h>

namespace trajopt_sqp
{
/**
 * @brief A simple SQP Solver that uses the QPSolver passed in
 */
class TrustRegionSQPSolver
{
public:
  using Ptr = std::shared_ptr<TrustRegionSQPSolver>;
  using ConstPtr = std::shared_ptr<const TrustRegionSQPSolver>;

  TrustRegionSQPSolver(QPSolver::Ptr qp_solver);

  bool init(QPProblem::Ptr qp_prob);

  void solve(QPProblem::Ptr qp_prob);

  /**
   * @brief Run a single convexification step which calls runTrustRegionLoop
   * @warning This should not normally be call directly, but exposed for online planning
   * @return True the QP solve converged, but does not mean the SQP solver has converged.
   */
  bool stepSQPSolver();

  /**
   * @brief Check if the SQPSolver constraints are satisfied
   * @warning This should not normally be call directly, but exposed for online planning
   */
  bool verifySQPSolverConvergence();

  /**
   * @brief The SQPSolver reported convergence but the constraints are not satisfied so this function
   * is used to increases the penalty on the constraints.
   */
  void adjustPenalty();

  /**
   * @brief Run trust region loop which calls stepOptimization adjusting box size
   * @warning This should not normally be call directly, but exposed for online planning
   */
  void runTrustRegionLoop();

  /**
   * @brief Solve the current QP Problem, storing the results and calling callbacks
   * @warning This should not normally be call directly, but exposed for online planning
   * @return SQP Status
   */
  SQPStatus solveQPProblem();

  /**
   * @brief Set the trust region box size
   * @warning This should not normally be call directly, but exposed for online planning
   * @param box_size The box size
   */
  void setBoxSize(double box_size);

  /**
   * @brief Calls all registered callbacks with the current state of of the problem
   * @return Returns false if any single callback returned false
   */
  bool callCallbacks();

  /** @brief Prints info about the current state of of the optimization */
  void printStepInfo() const;

  /** @brief Registers an optimization callback */
  void registerCallback(const SQPCallback::Ptr& callback);

  /** @brief If true then debug information will be printed to the terminal */
  bool verbose{ false };

  /** @brief Contains parameters that control the SQP optimization */
  SQPParameters params;

  /** @brief Gets the optimization status (currently unset) */
  const SQPStatus& getStatus();

  /** @brief Gets the SQP optimization results */
  const SQPResults& getResults();

  /** @brief The QP Solver used to solve a single step of the SQP routine  */
  QPSolver::Ptr qp_solver;

  /** @brief The QP problem created from the NLP */
  QPProblem::Ptr qp_problem;

protected:
  SQPStatus status_;
  SQPResults results_;
  std::vector<SQPCallback::Ptr> callbacks_;
};

}  // namespace trajopt_sqp
#endif
