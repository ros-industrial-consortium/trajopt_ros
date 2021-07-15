#ifndef TRAJOPT_SQP_QP_PROBLEM_BASE_H
#define TRAJOPT_SQP_QP_PROBLEM_BASE_H

#include <memory>
#include <trajopt_sqp/types.h>

namespace trajopt_sqp
{
/** @brief QP Problem Base */
class QPProblem
{
public:
  using Ptr = std::shared_ptr<QPProblem>;
  using ConstPtr = std::shared_ptr<const QPProblem>;

  /** @brief Set the current Optimization variables */
  virtual void setVariables(const double* x) = 0;

  /** @brief Set the current Optimization variable values */
  virtual Eigen::VectorXd getVariableValues() const = 0;

  /** @brief Run the full convexification routine */
  virtual void convexify() = 0;

  //  /**
  //   * @brief Helper that updates the cost QP hessian
  //   * @details Called by convexify()
  //   */
  //  virtual void updateHessian() = 0;

  //  /**
  //   * @brief Helper that updates the cost gradient including slack variables
  //   * @details Called by convexify()
  //   */
  //  virtual void updateGradient() = 0;

  //  /**
  //   * @brief Helper that linearizes the constraints about the current point, storing the
  //   * jacobian as the constraint matrix and adding slack variables.
  //   * @details Called by convexify()
  //   */
  //  virtual void linearizeConstraints() = 0;

  //  /**
  //   * @brief Helper that updates the NLP constraint bounds (top section)
  //   * @details Called by convexify()
  //   */
  //  virtual void updateNLPConstraintBounds() = 0;

  //  /**
  //   * @brief Helper that updates the NLP variable bounds (middle section)
  //   * @details Called by convexify()
  //   */
  //  virtual void updateNLPVariableBounds() = 0;

  //  /**
  //   * @brief Called by convexify() - helper that updates the slack variable bounds (bottom section)
  //   * @details A slack variable is referred to as an additional variable that has been introduced
  //   * to the optimization problem to turn a inequality constraint into an equality constraint.
  //   * Information taken from: https://en.wikipedia.org/wiki/Slack_variable
  //   *
  //   * As with the other variables in the augmented constraints, the slack variable cannot take on
  //   * negative values, as the simplex algorithm requires them to be positive or zero.
  //   *
  //   *   - If a slack variable associated with a constraint is zero at a particular candidate solution,
  //   *     the constraint is binding there, as the constraint restricts the possible changes from that point.
  //   *   - If a slack variable is positive at a particular candidate solution, the constraint is non-binding
  //   *     there, as the constraint does not restrict the possible changes from that point.
  //   *   - If a slack variable is negative at some point, the point is infeasible (not allowed), as it does
  //   *     not satisfy the constraint.
  //   *
  //   * Terminology
  //   *
  //   *   - If an inequality constraint holds with equality at the optimal point, the constraint is said to
  //   *     be binding, as the point cannot be varied in the direction of the constraint even though doing
  //   *     so would improve the value of the objective function.
  //   *   - If an inequality constraint holds as a strict inequality at the optimal point (that is, does
  //   *     not hold with equality), the constraint is said to be non-binding, as the point could be varied
  //   *     in the direction of the constraint, although it would not be optimal to do so. Under certain
  //   *     conditions, as for example in convex optimization, if a constraint is non-binding, the optimization
  //   *     problem would have the same solution even in the absence of that constraint.
  //   *   - If a constraint is not satisfied at a given point, the point is said to be infeasible.
  //   *
  //   * @example By introducing the slack variable y >= 0, the inequality Ax <= b can be converted
  //   * to the equation Ax + y = b.
  //   */
  //  virtual void updateSlackVariableBounds() = 0;

  /**
   * @brief Evaluates the cost of the convexified function (ie using the stored gradient and hessian) at var_vals
   * @param var_vals Point at which the convex cost is calculated. Should be size num_qp_vars
   */
  virtual double evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals) = 0;

  /**
   * @brief Evaluated the cost of the convexified function (ie using the stored gradient and hessian) at var_vals
   * @note This will be relatively computationally expensive, as we will have to loop through all the cost components in
   * the problem and calculate their values manually.
   * @param var_vals Point at which the convex cost is calculated. Should be size num_qp_vars
   * @return Cost associated with each cost term in the problem (for debugging)
   */
  virtual Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) = 0;

  /**
   * @brief Evaluates the sum of the cost functions at var_vals
   * @note This will be relatively computationally expensive, as we will have to loop through all the cost components in
   * the problem and calculate their values manually.
   * @param var_vals Point at which the cost is calculated. Should be size num_qp_vars
   */
  virtual double evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals) = 0;

  /**
   * @brief Evaluated the cost functions at var_vals
   * @note This will be relatively computationally expensive, as we will have to loop through all the cost components in
   * the problem and calculate their values manually.
   * @param var_vals Point at which the cost is calculated. Should be size num_qp_vars
   * @return Cost associated with each cost term in the problem (for debugging)
   */
  virtual Eigen::VectorXd evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) = 0;

  /**
   * @brief get the current NLP costs.
   * @return Vector of costs.
   */
  virtual Eigen::VectorXd getExactCosts() = 0;

  /**
   * @brief Evaluated the costraint violation of the convexified function (ie using the stored constraint matrix) at
   * var_vals
   * @param var_vals
   * @return
   */
  virtual Eigen::VectorXd evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) = 0;

  /**
   * @brief Evaluate NLP constraint violations at the provided var_vals. Values > 0 are violations
   * @return Vector of constraint violations. Values > 0 are violations
   */
  virtual Eigen::VectorXd evaluateExactConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) = 0;

  /**
   * @brief get the current NLP constraint violations. Values > 0 are violations
   * @return Vector of constraint violations. Values > 0 are violations
   */
  virtual Eigen::VectorXd getExactConstraintViolations() = 0;
  /**
   * @brief Uniformly scales the box size  (box_size_ = box_size_ * scale)
   * @param scale Value by which the box size is scaled
   */
  virtual void scaleBoxSize(double& scale) = 0;
  /**
   * @brief Sets the box size to the input vector
   * @param box_size New box size
   */
  virtual void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) = 0;
  /**
   * @brief Returns the box size
   * @return The box size for each variable
   */
  virtual Eigen::VectorXd getBoxSize() const = 0;

  /** @brief Prints all members to the terminal in a human readable form */
  virtual void print() const = 0;

  virtual Eigen::Index getNumNLPVars() const = 0;
  virtual Eigen::Index getNumNLPConstraints() const = 0;
  virtual Eigen::Index getNumNLPCosts() const = 0;
  virtual Eigen::Index getNumQPVars() const = 0;
  virtual Eigen::Index getNumQPConstraints() const = 0;

  virtual const std::vector<std::string>& getNLPConstraintNames() const = 0;
  virtual const std::vector<std::string>& getNLPCostNames() const = 0;

  virtual const Eigen::Ref<const Eigen::VectorXd> getBoxSize() = 0;
  virtual const Eigen::Ref<const Eigen::VectorXd> getConstraintMeritCoeff() = 0;

  virtual const Eigen::Ref<const Eigen::SparseMatrix<double>> getHessian() = 0;
  virtual const Eigen::Ref<const Eigen::VectorXd> getGradient() = 0;

  virtual const Eigen::Ref<const Eigen::SparseMatrix<double>> getConstraintMatrix() = 0;
  virtual const Eigen::Ref<const Eigen::VectorXd> getBoundsLower() = 0;
  virtual const Eigen::Ref<const Eigen::VectorXd> getBoundsUpper() = 0;
};

}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_QP_PROBLEM_BASE_H
