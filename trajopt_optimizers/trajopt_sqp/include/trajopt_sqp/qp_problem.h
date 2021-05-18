/**
 * @file qp_problem.h
 * @brief Converts general NLP to QP for SQP routine
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
#ifndef TRAJOPT_SQP_INCLUDE_QP_PROBLEM_H_
#define TRAJOPT_SQP_INCLUDE_QP_PROBLEM_H_

#include <memory>
#include <trajopt_sqp/types.h>
#include <ifopt/problem.h>

namespace trajopt_sqp
{
enum class ConstraintType
{
  EQ,
  INEQ
};

/** @brief Converts a general NLP into a convexified QP that can be solved by a QP solver */
class QPProblem
{
public:
  using Ptr = std::shared_ptr<QPProblem>;
  using ConstPtr = std::shared_ptr<const QPProblem>;

  /** @brief Sets up the problem and initializes matrices
   * @param nlp
   */
  void init(ifopt::Problem& nlp);
  /** @brief Run the full convexification routine. If in doubt, init(nlp) then convexify() */
  void convexify();

  /** @brief Called by convexify() - helper that updates the cost hessian (Currently does nothing since IFOPT doesn't
   * support Hessians*/
  void updateHessian();
  /** @brief Called by convexify() - helper that updates the cost gradient including slack variables */
  void updateGradient();
  /** @brief Called by convexify() - helper that linearizes the constraints about the current point, storing the
   * jacobian as the constraint matrix and adding slack variables.*/
  void linearizeConstraints();
  /** @brief Called by convexify() - helper that updates the NLP constraint bounds (top section) */
  void updateNLPConstraintBounds();
  /** @brief Called by convexify() - helper that updates the NLP variable bounds (middle section) */
  void updateNLPVariableBounds();
  /** @brief Called by convexify() - helper that updates the slack variable bounds (bottom section) */
  void updateSlackVariableBounds();

  /**
   * @brief Evaluates the cost of the convexified function (ie using the stored gradient and hessian) at var_vals
   * @param var_vals Point at which the convex cost is calculated. Should be size num_qp_vars
   */
  double evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

  /** @brief TODO: This is unimplemented, but it will return the cost associated with each of the costs. Note this will
   * be relatively computationally expensive, as we will have to loop through all the cost components in the problem and
   * calculate their values manually.
   * @param var_vals Point at which the convex cost is calculated. Should be size num_qp_vars
   * @return Cost associated with each cost term in the problem (for debugging)
   */
  Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

  Eigen::VectorXd evaluateConvexConstraintViolation(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

  /**
   * @brief Evaluate NLP constraint violations at the provided var_vals. Values > 0 are violations
   * @return Vector of constraint violations. Values > 0 are violations
   */
  Eigen::VectorXd evaluateExactConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals);

  /**
   * @brief get the current NLP constraint violations. Values > 0 are violations
   * @return Vector of constraint violations. Values > 0 are violations
   */
  Eigen::VectorXd getExactConstraintViolations();
  /**
   * @brief Uniformly scales the box size  (box_size_ = box_size_ * scale)
   * @param scale Value by which the box size is scaled
   */
  void scaleBoxSize(double& scale);
  /**
   * @brief Sets the box size to the input vector
   * @param box_size New box size
   */
  void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size);
  /**
   * @brief Returns the box size
   * @return The box size for each variable
   */
  Eigen::VectorXd getBoxSize() const;

  /** @brief Prints all members to the terminal in a human readable form */
  void print() const;

  const Eigen::Index& getNumNLPVars() { return num_nlp_vars_; };
  const Eigen::Index& getNumNLPConstraints() { return num_nlp_cnts_; };
  const Eigen::Index& getNumNLPCosts() { return num_nlp_costs_; };
  const Eigen::Index& getNumQPVars() { return num_qp_vars_; };
  const Eigen::Index& getNumQPConstraints() { return num_qp_cnts_; };

  const Eigen::Ref<const Eigen::VectorXd> getBoxSize() { return box_size_; };
  const Eigen::Ref<const Eigen::VectorXd> getConstraintMeritCoeff() { return constraint_merit_coeff_; };

  const Eigen::Ref<const Eigen::SparseMatrix<double>> getHessian() { return hessian_; };
  const Eigen::Ref<const Eigen::VectorXd> getGradient() { return gradient_; };

  const Eigen::Ref<const Eigen::SparseMatrix<double>> getConstraintMatrix() { return constraint_matrix_; };
  const Eigen::Ref<const Eigen::VectorXd> getBoundsLower() { return bounds_lower_; };
  const Eigen::Ref<const Eigen::VectorXd> getBoundsUpper() { return bounds_upper_; };

protected:
  ifopt::Problem* nlp_;

  Eigen::Index num_nlp_vars_;
  Eigen::Index num_nlp_cnts_;
  Eigen::Index num_nlp_costs_;
  Eigen::Index num_qp_vars_;
  Eigen::Index num_qp_cnts_;
  std::vector<ConstraintType> constraint_types_;

  /** @brief Box size - constraint is set at current_val +/- box_size */
  Eigen::VectorXd box_size_;
  Eigen::VectorXd constraint_merit_coeff_;

  Eigen::SparseMatrix<double> hessian_;
  Eigen::VectorXd gradient_;
  Eigen::VectorXd cost_constant_;

  Eigen::SparseMatrix<double> constraint_matrix_;
  Eigen::VectorXd bounds_lower_;
  Eigen::VectorXd bounds_upper_;
  // This should be the center of the bounds
  Eigen::VectorXd constraint_constant_;

  /** @brief This calculates the constant expression in the quadratic expression for the constraints */
  void updateConstraintsConstantExpression();

  /** @brief This calculates the constant expression in the quadratic expression for the costs */
  void updateCostsConstantExpression();

  /** @brief This will calculate contraint violations given constraint values */
  Eigen::VectorXd calcConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& cnt_vals);
};

}  // namespace trajopt_sqp

#endif
