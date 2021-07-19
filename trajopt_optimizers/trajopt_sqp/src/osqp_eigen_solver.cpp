/**
 * @file osqp_eigen_solver.cpp
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
#include <trajopt_sqp/osqp_eigen_solver.h>

#include <OsqpEigen/OsqpEigen.h>

namespace trajopt_sqp
{
OSQPEigenSolver::OSQPEigenSolver()
{
  if (verbosity > 0)
    solver_.settings()->setVerbosity(true);
  else
    solver_.settings()->setVerbosity(false);
  solver_.settings()->setWarmStart(true);
  solver_.settings()->setPolish(true);
  solver_.settings()->setAdaptiveRho(false);
  solver_.settings()->setMaxIteration(8192);
  solver_.settings()->setAbsoluteTolerance(1e-4);
  solver_.settings()->setRelativeTolerance(1e-6);
}

bool OSQPEigenSolver::init(Eigen::Index num_vars, Eigen::Index num_cnts)
{
  // Set the solver size
  num_cnts_ = num_cnts;
  num_vars_ = num_vars;
  solver_.data()->setNumberOfVariables(static_cast<int>(num_vars_));
  solver_.data()->setNumberOfConstraints(static_cast<int>(num_cnts_));

  solver_status_ = QPSolverStatus::INITIALIZED;

  return true;
}

bool OSQPEigenSolver::clear()
{
  // Clear all data
  solver_.clearSolver();
  solver_.data()->clearHessianMatrix();
  solver_.data()->clearLinearConstraintsMatrix();
  return true;
}

bool OSQPEigenSolver::solve()
{
  // In order to call initSolver, everything must have already been set, so we call it right before solving
  if (!solver_.isInitialized())
    solver_.initSolver();
  if (solver_.solve())
    return true;

  if (verbosity > 0)
  {
    // If primal infeasible
    if (solver_.workspace()->info->status_val == -3)
    {
      Eigen::Map<Eigen::VectorXd> primal_certificate(solver_.workspace()->delta_x, num_cnts_, 1);
      std::cout << "OSQP Status: " << solver_.workspace()->info->status << std::endl;
      std::cout << "\n---------------------------------------\n";
      std::cout << std::scientific << "Primal Certificate (v): " << primal_certificate.transpose() << std::endl;

      double first = bounds_lower_.transpose() * primal_certificate;
      double second = bounds_upper_.transpose() * primal_certificate;

      std::cout << "A.transpose() * v = 0\n"
                << "l.transpose() * v = " << first << "    u.transpose() * v = " << second << std::endl;
      std::cout << "l.transpose() * v + u.transpose() * v  = " << first + second << " < 0\n";
      std::cout << "Bounds_lower: " << bounds_lower_.transpose() << std::endl;
      std::cout << "Bounds_upper: " << bounds_upper_.transpose() << std::endl;
      std::cout << std::fixed;
      std::cout << "\n---------------------------------------\n";
    }

    // If dual infeasible
    if (solver_.workspace()->info->status_val == -4)
    {
      Eigen::Map<Eigen::VectorXd> dual_certificate(solver_.workspace()->delta_y, num_vars_, 1);
      std::cout << "OSQP Status: " << solver_.workspace()->info->status << std::endl;
      std::cout << "\n---------------------------------------\n";
      std::cout << "Dual Certificate (x): " << dual_certificate.transpose() << std::endl;

      // TODO: Find a way to get the hessian here
      //        std::cout << "P*x = " << (qp_problem_->getHessian() * dual_certificate).transpose() << " = 0" <<
      //        std::endl;
      std::cout << "q.transpose() * x = " << gradient_.transpose() * dual_certificate << " < 0" << std::endl;
      std::cout << std::fixed;
      std::cout << "\n---------------------------------------\n";
    }
  }

  solver_status_ = QPSolverStatus::QP_ERROR;
  return false;
}

Eigen::VectorXd OSQPEigenSolver::getSolution()
{
  Eigen::VectorXd solution = solver_.getSolution();
  return solution;
}

bool OSQPEigenSolver::updateHessianMatrix(const SparseMatrix& hessian)
{
  // Clean up values close to 0
  // Also multiply by 2 because OSQP is multiplying by (1/2) for the objective fuction
  SparseMatrix cleaned = 2.0 * hessian.pruned(1e-7);

  if (solver_.isInitialized())
    return solver_.updateHessianMatrix(cleaned);

  solver_.data()->clearHessianMatrix();
  return solver_.data()->setHessianMatrix(cleaned);
}

bool OSQPEigenSolver::updateGradient(const Eigen::Ref<const Eigen::VectorXd>& gradient)
{
  gradient_ = gradient;

  if (solver_.isInitialized())
    return solver_.updateGradient(gradient_);

  return solver_.data()->setGradient(gradient_);
}

bool OSQPEigenSolver::updateLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lowerBound)
{
  bounds_lower_ = lowerBound.cwiseMax(Eigen::VectorXd::Ones(num_cnts_) * -OSQP_INFTY);
  return solver_.updateLowerBound(bounds_lower_);
}

bool OSQPEigenSolver::updateUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upperBound)
{
  bounds_upper_ = upperBound.cwiseMin(Eigen::VectorXd::Ones(num_cnts_) * OSQP_INFTY);
  return solver_.updateUpperBound(bounds_upper_);
}

bool OSQPEigenSolver::updateBounds(const Eigen::Ref<const Eigen::VectorXd>& lowerBound,
                                   const Eigen::Ref<const Eigen::VectorXd>& upperBound)
{
  bounds_lower_ = lowerBound.cwiseMax(Eigen::VectorXd::Ones(num_cnts_) * -OSQP_INFTY);
  bounds_upper_ = upperBound.cwiseMin(Eigen::VectorXd::Ones(num_cnts_) * OSQP_INFTY);

  if (solver_.isInitialized())
    return solver_.updateBounds(bounds_lower_, bounds_upper_);

  bool success = solver_.data()->setLowerBound(bounds_lower_);
  success &= solver_.data()->setUpperBound(bounds_upper_);
  return success;
}

bool OSQPEigenSolver::updateLinearConstraintsMatrix(const SparseMatrix& linearConstraintsMatrix)
{
  assert(num_cnts_ == linearConstraintsMatrix.rows());
  assert(num_vars_ == linearConstraintsMatrix.cols());

  solver_.data()->clearLinearConstraintsMatrix();
  SparseMatrix cleaned = linearConstraintsMatrix.pruned(1e-7);

  if (solver_.isInitialized())
    return solver_.updateLinearConstraintsMatrix(linearConstraintsMatrix);

  return solver_.data()->setLinearConstraintsMatrix(cleaned);
}

}  // namespace trajopt_sqp
