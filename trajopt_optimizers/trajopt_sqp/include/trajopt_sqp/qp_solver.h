/**
 * @file qp_solver.h
 * @brief Base class for QP solvers
 *
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug This is not being used currently. The OSQPEigen solver interface needs to be cleaned up such that this base
 * class can be used in trust_region_sqp_solver.h
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
#ifndef TRAJOPT_SQP_INCLUDE_QP_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_QP_SOLVER_H_

#include <trajopt_sqp/types.h>

namespace trajopt_sqp
{
enum class QPSolverStatus
{
  UNITIALIZED,
  INITIALIZED,
  QP_ERROR
};

/**
 * @brief Base class for Quadratic programming solvers. The general form for these problems is
 * minimize(0.5 * x.transpose() * P * x + q.transpose * x)
 * subject to lower_bound <= A * x <= upper_bound
 *
 * Where P is the hessian matrix, q is the gradient vector, and A is the linear constraint matrix.
 */
class QPSolver
{
public:
  using Ptr = std::shared_ptr<QPSolver>;
  using ConstPtr = std::shared_ptr<const QPSolver>;

  QPSolver() = default;
  virtual ~QPSolver() = default;
  QPSolver(const QPSolver&) = default;
  QPSolver& operator=(const QPSolver&) = default;
  QPSolver(QPSolver&&) = default;
  QPSolver& operator=(QPSolver&&) = default;

  /**
   * @brief Initializes the QP solver. This is called prior to any of the update methods
   * @param num_vars Number of QP variables
   * @param num_cnts Number of QP constraints
   * @return true if successful
   */
  virtual bool init(Eigen::Index num_vars, Eigen::Index num_cnts) = 0;

  /**
   * @brief Clears the QP solver
   * @return true if successful
   */
  virtual bool clear() = 0;

  /**
   * @brief Solves the QP
   * @return true if successful
   */
  virtual bool solve() = 0;

  /**
   * @brief Gets the solution
   * @return The found solution
   */
  virtual Eigen::VectorXd getSolution() = 0;

  /**
   * @brief Updates the cost hessian
   * @param hessian The QP hessian. Should be n_vars x n_vars
   * @return true if successful
   */
  virtual bool updateHessianMatrix(const SparseMatrix& hessian) = 0;

  /**
   * @brief Updates the cost gradient
   * @param gradient The QP gradient. Should be n_vars x 1
   * @return true if successful
   */
  virtual bool updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient) = 0;

  /**
   * @brief Updates the constraint lower bound
   * @param lowerBound The constrait bound. Should be n_constraints x 1
   * @return true if succesful
   */
  virtual bool updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound) = 0;

  /**
   * @brief Updates the constraint upper bound
   * @param upperBound The constraint bound. Should be n_constraints x 1
   * @return true if successful
   */
  virtual bool updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound) = 0;

  /**
   * @brief Updates both constraint bounds
   * @param lowerBound The lower constraint bound. Should be n_constraints x 1
   * @param upperBound The upper constraint bound. Should be n_constraints x 1
   * @return true if successful
   */
  virtual bool updateBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                            const Eigen::Ref<const Eigen::VectorXd>& upperBound) = 0;

  /**
   * @brief Updates the linear constraint matrix
   * @param linearConstraintsMatrix Input constraint matrix
   * @return true if successful
   */
  virtual bool updateLinearConstraintsMatrix(const SparseMatrix& linearConstraintsMatrix) = 0;

  /**
   * @brief Returns the solver status
   * @return The solver status
   */
  virtual QPSolverStatus getSolverStatus() const = 0;

  /** @brief verbose if > 0. Default: 0 */
  int verbosity{ 0 };
};
}  // namespace trajopt_sqp

#endif
