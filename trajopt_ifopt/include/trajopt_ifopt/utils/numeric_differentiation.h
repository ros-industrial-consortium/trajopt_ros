/**
 * @file numeric_differentiation.h
 * @brief Contains numeric differentiation code
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
#ifndef TRAJOPT_IFOPT_NUMERIC_DIFFERENTIATION_H
#define TRAJOPT_IFOPT_NUMERIC_DIFFERENTIATION_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <ifopt/cost_term.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_ifopt
{
using SparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;
using ErrorCalculator = std::function<Eigen::VectorXd(const Eigen::Ref<const Eigen::VectorXd>&)>;
using JacobianCalculator = std::function<SparseMatrix(const Eigen::Ref<const Eigen::VectorXd>&)>;

/**
 * @brief Calculates the jacobian of the given error calculator using forward numeric differentiation
 * @param f Input error calculator, Eigen::VectorXd(const Eigen::Ref<const Eigen::VectorXd>&)
 * @param x Point about which f is calculated
 * @param epsilon Amount x is perturbed
 * @return The resulting jacobian. If f(x) = y, jac.size = [y.size(), x.size()]
 */
inline SparseMatrix calcForwardNumJac(const ErrorCalculator& f,
                                      const Eigen::Ref<const Eigen::VectorXd>& x,
                                      double epsilon)
{
  Eigen::VectorXd y = f(x);
  Eigen::MatrixXd out(y.size(), x.size());
  Eigen::VectorXd x_perturbed = x;
  for (int i = 0; i < x.size(); ++i)
  {
    x_perturbed(i) = x(i) + epsilon;
    Eigen::VectorXd y_perturbed = f(x_perturbed);
    out.col(i) = (y_perturbed - y) / epsilon;
    x_perturbed(i) = x(i);
  }

  return out.sparseView();
}

// inline std::vector<SparseMatrix> calcForwardNumHessian(const JacobianCalculator& f, const Eigen::Ref<const
// Eigen::VectorXd>& x, double epsilon)
//{
//  SparseMatrix y = f(x);
//  Eigen::MatrixXd out(x.size(), x.size());
//  Eigen::VectorXd x_perturbed = x;
//  for (int i = 0; i < x.size(); ++i)
//  {
//    x_perturbed(i) = x(i) + epsilon;
//    SparseMatrix y_perturbed = f(x_perturbed);
//    out.col(i) = (y_perturbed - y) / epsilon;
//    x_perturbed(i) = x(i);
//  }
//}

// inline calcGradAndDiagHess(const ErrorCalculator& f,
//                           const Eigen::Ref<const Eigen::VectorXd>& x,
//                           double epsilon,
//                           double& y,
//                           Eigen::Ref<Eigen::VectorXd> grad,
//                           Eigen::Ref<SparseMatrix> hess)
//{
//  y = f(x);
//  grad.resize(x.size());
//  hess.resize(x.size(), x.size());
//  hess.reserve(x.size());
//  Eigen::VectorXd xpert = x;
//  for (int i = 0; i < x.size(); ++i)
//  {
//    xpert(i) = x(i) + epsilon / 2;
//    double yplus = f(xpert);
//    xpert(i) = x(i) - epsilon / 2;
//    double yminus = f(xpert);
//    grad(i) = (yplus - yminus) / epsilon;
//    hess(i) = (yplus + yminus - 2 * y) / (epsilon * epsilon / 4);
//    xpert(i) = x(i);
//  }
//}

// void calcGradAndFullHess(const ErrorCalculator& f,
//                         const Eigen::Ref<const Eigen::VectorXd>& x,
//                         double epsilon,
//                         double& y,
//                         Eigen::Ref<Eigen::VectorXd> grad,
//                         Eigen::Ref<SparseMatrix> hess)
//{
//  y = f(x);
//  VectorOfVector::Ptr grad_func = forwardNumGrad(f, epsilon);
//  grad = grad_func->call(x);
//  hess = calcForwardNumJac(*grad_func, x, epsilon);
//  hess = (hess + hess.transpose()) / 2;
//}
}  // namespace trajopt_ifopt

#endif
