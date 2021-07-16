/**
 * @file ifopt_qp_problem.h
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
#ifndef TRAJOPT_SQP_IFOPT_QP_PROBLEM_H_
#define TRAJOPT_SQP_IFOPT_QP_PROBLEM_H_

#include <memory>
#include <trajopt_sqp/types.h>
#include <ifopt/problem.h>
#include <trajopt_sqp/qp_problem.h>

namespace trajopt_sqp
{
/** @brief Converts a general NLP into a convexified QP that can be solved by a QP solver */
class IfoptQPProblem : public QPProblem
{
public:
  using Ptr = std::shared_ptr<IfoptQPProblem>;
  using ConstPtr = std::shared_ptr<const IfoptQPProblem>;

  IfoptQPProblem() = default;
  IfoptQPProblem(ifopt::Problem& nlp);

  /**
   * @brief Sets up the problem and initializes matrices
   * @param nlp
   */
  void init(ifopt::Problem& nlp);

  void setVariables(const double* x) override;
  Eigen::VectorXd getVariableValues() const override;

  void convexify() override;
  void updateHessian() override;
  void updateGradient() override;
  void linearizeConstraints() override;
  void updateNLPConstraintBounds() override;
  void updateNLPVariableBounds() override;
  void updateSlackVariableBounds() override;

  double evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals) override;

  /**
   * @brief Evaluated the cost of the convexified function (ie using the stored gradient and hessian) at var_vals
   * @note This will be relatively computationally expensive, as we will have to loop through all the cost components in
   * the problem and calculate their values manually.
   * @param var_vals Point at which the convex cost is calculated. Should be size num_qp_vars
   * @return Cost associated with each cost term in the problem (for debugging)
   */
  Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) override;

  /**
   * @brief Evaluates the sum of the cost functions at var_vals
   * @note This will be relatively computationally expensive, as we will have to loop through all the cost components in
   * the problem and calculate their values manually.
   * @param var_vals Point at which the cost is calculated. Should be size num_qp_vars
   */
  double evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals) override;

  /**
   * @brief Evaluated the cost functions at var_vals
   * @note This will be relatively computationally expensive, as we will have to loop through all the cost components in
   * the problem and calculate their values manually.
   * @param var_vals Point at which the cost is calculated. Should be size num_qp_vars
   * @return Cost associated with each cost term in the problem (for debugging)
   */
  Eigen::VectorXd evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) override;

  /**
   * @brief get the current NLP costs.
   * @return Vector of costs.
   */
  Eigen::VectorXd getExactCosts() override;

  /**
   * @brief Evaluated the costraint violation of the convexified function (ie using the stored constraint matrix) at
   * var_vals
   * @param var_vals
   * @return
   */
  Eigen::VectorXd evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) override;

  /**
   * @brief Evaluate NLP constraint violations at the provided var_vals. Values > 0 are violations
   * @return Vector of constraint violations. Values > 0 are violations
   */
  Eigen::VectorXd evaluateExactConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) override;

  /**
   * @brief get the current NLP constraint violations. Values > 0 are violations
   * @return Vector of constraint violations. Values > 0 are violations
   */
  Eigen::VectorXd getExactConstraintViolations() override;
  /**
   * @brief Uniformly scales the box size  (box_size_ = box_size_ * scale)
   * @param scale Value by which the box size is scaled
   */
  void scaleBoxSize(double& scale) override;
  /**
   * @brief Sets the box size to the input vector
   * @param box_size New box size
   */
  void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) override;
  /**
   * @brief Returns the box size
   * @return The box size for each variable
   */
  Eigen::VectorXd getBoxSize() const override;

  /** @brief Prints all members to the terminal in a human readable form */
  void print() const override;

  Eigen::Index getNumNLPVars() const override { return num_nlp_vars_; }
  Eigen::Index getNumNLPConstraints() const override { return num_nlp_cnts_; }
  Eigen::Index getNumNLPCosts() const override { return num_nlp_costs_; }
  Eigen::Index getNumQPVars() const override { return num_qp_vars_; }
  Eigen::Index getNumQPConstraints() const override { return num_qp_cnts_; }

  const std::vector<std::string>& getNLPConstraintNames() const override { return constraint_names_; }
  const std::vector<std::string>& getNLPCostNames() const override { return cost_names_; }

  const Eigen::Ref<const Eigen::VectorXd> getBoxSize() override { return box_size_; }
  const Eigen::Ref<const Eigen::VectorXd> getConstraintMeritCoeff() override { return constraint_merit_coeff_; }

  const Eigen::Ref<const Eigen::SparseMatrix<double>> getHessian() override { return hessian_; }
  const Eigen::Ref<const Eigen::VectorXd> getGradient() override { return gradient_; }

  const Eigen::Ref<const Eigen::SparseMatrix<double>> getConstraintMatrix() override { return constraint_matrix_; }
  const Eigen::Ref<const Eigen::VectorXd> getBoundsLower() override { return bounds_lower_; }
  const Eigen::Ref<const Eigen::VectorXd> getBoundsUpper() override { return bounds_upper_; }

protected:
  ifopt::Problem* nlp_;

  Eigen::Index num_nlp_vars_;
  Eigen::Index num_nlp_cnts_;
  Eigen::Index num_nlp_costs_;
  Eigen::Index num_qp_vars_;
  Eigen::Index num_qp_cnts_;
  std::vector<std::string> constraint_names_;
  std::vector<std::string> cost_names_;

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
};

}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_IFOPT_QP_PROBLEM_H_
