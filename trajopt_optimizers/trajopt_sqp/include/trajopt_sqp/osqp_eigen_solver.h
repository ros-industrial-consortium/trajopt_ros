/**
 * @file osqp_eigen_solver.h
 * @brief Interface to the OSQPEigen solver
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
#ifndef TRAJOPT_SQP_INCLUDE_OSQP_EIGEN_SOLVER_H_
#define TRAJOPT_SQP_INCLUDE_OSQP_EIGEN_SOLVER_H_

#include <ifopt/problem.h>
#include <trajopt_sqp/qp_solver.h>
#include <OsqpEigen/OsqpEigen.h>

namespace trajopt_sqp
{
/**
 * @brief An Interface to the OSQPEigen QP Solver
 */
class OSQPEigenSolver : public QPSolver
{
public:
  using Ptr = std::shared_ptr<OSQPEigenSolver>;
  using ConstPtr = std::shared_ptr<const OSQPEigenSolver>;

  OSQPEigenSolver();

  bool init(Eigen::Index num_vars, Eigen::Index num_cnts) override;

  bool clear() override;

  bool solve() override;

  Eigen::VectorXd getSolution() override;

  bool updateHessianMatrix(const SparseMatrix& hessian) override;

  bool updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient) override;

  bool updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound) override;

  bool updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound) override;

  bool updateBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                    const Eigen::Ref<const Eigen::VectorXd>& upperBound) override;

  bool updateLinearConstraintsMatrix(const SparseMatrix& linearConstraintsMatrix) override;

  QPSolverStatus getSolverStatus() const override { return solver_status_; };

  OsqpEigen::Solver solver_;

private:
  // Depending on what they decide to do with this issue, these could be dropped
  // https://github.com/robotology/osqp-eigen/issues/17
  Eigen::VectorXd bounds_lower_;
  Eigen::VectorXd bounds_upper_;
  Eigen::VectorXd gradient_;
  Eigen::Index num_vars_;
  Eigen::Index num_cnts_;

  QPSolverStatus solver_status_{ QPSolverStatus::UNITIALIZED };
};

}  // namespace trajopt_sqp

#endif
