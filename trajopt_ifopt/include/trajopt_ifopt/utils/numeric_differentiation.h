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
using ErrorCalculator = std::function<Eigen::VectorXd(const Eigen::Ref<const Eigen::VectorXd>&)>;
using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

/**
 * @brief Calculates the jacobian of the given error calculator using forward numeric differentiation
 * @param f Input error calculator, Eigen::VectorXd(const Eigen::Ref<const Eigen::VectorXd>&)
 * @param x Point about which f is calculated
 * @param epsilon Amount x is perturbed
 * @return The resulting jacobian. If f(x) = y, jac.size = [y.size(), x.size()]
 */
inline Jacobian calcForwardNumJac(const ErrorCalculator& f, const Eigen::Ref<const Eigen::VectorXd>& x, double epsilon)
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
}  // namespace trajopt_ifopt

#endif
