#ifndef TRAJOPT_SQP_TRAJOPT_QP_PROBLEM_H
#define TRAJOPT_SQP_TRAJOPT_QP_PROBLEM_H

#include <trajopt_sqp/qp_problem.h>
#include <trajopt_sqp/expressions.h>
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>

namespace trajopt_sqp
{
/** @brief Converts a general NLP into a convexified QP that can be solved by a QP solver */
class TrajOptQPProblem : public QPProblem
{
public:
  using Ptr = std::shared_ptr<TrajOptQPProblem>;
  using ConstPtr = std::shared_ptr<const TrajOptQPProblem>;

  TrajOptQPProblem();

  void addVariableSet(ifopt::VariableSet::Ptr variable_set) override;

  void addConstraintSet(ifopt::ConstraintSet::Ptr constraint_set) override;

  void addCostSet(ifopt::ConstraintSet::Ptr constraint_set, CostPenaltyType penalty_type) override;

  void setup() override;

  void setVariables(const double* x) override;

  Eigen::VectorXd getVariableValues() const override;

  void convexify() override;

  double evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals) override;

  Eigen::VectorXd evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) override;

  double evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals) override;

  Eigen::VectorXd evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals) override;

  Eigen::VectorXd getExactCosts() override;

  Eigen::VectorXd evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) override;

  Eigen::VectorXd evaluateExactConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals) override;

  Eigen::VectorXd getExactConstraintViolations() override;

  void scaleBoxSize(double& scale) override;

  void setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size) override;

  Eigen::VectorXd getBoxSize() const override;

  void print() const override;

  Eigen::Index getNumNLPVars() const override;
  Eigen::Index getNumNLPConstraints() const override;
  Eigen::Index getNumNLPCosts() const override;
  Eigen::Index getNumQPVars() const override;
  Eigen::Index getNumQPConstraints() const override;

  const std::vector<std::string>& getNLPConstraintNames() const override;
  const std::vector<std::string>& getNLPCostNames() const override;

  const Eigen::Ref<const Eigen::VectorXd> getBoxSize() override;
  const Eigen::Ref<const Eigen::VectorXd> getConstraintMeritCoeff() override;

  const Eigen::Ref<const SparseMatrix> getHessian() override;
  const Eigen::Ref<const Eigen::VectorXd> getGradient() override;

  const Eigen::Ref<const SparseMatrix> getConstraintMatrix() override;
  const Eigen::Ref<const Eigen::VectorXd> getBoundsLower() override;
  const Eigen::Ref<const Eigen::VectorXd> getBoundsUpper() override;

protected:
  bool initialized_{ false };
  ifopt::Composite::Ptr variables_;
  ifopt::Composite constraints_;
  ifopt::Composite squared_costs_;
  ifopt::Composite abs_costs_;
  ifopt::Composite hing_costs_;
  std::vector<ConstraintType> constraint_types_;

  Eigen::Index num_qp_vars_;
  Eigen::Index num_qp_cnts_;

  std::vector<std::string> constraint_names_;
  std::vector<std::string> cost_names_;

  /** @brief Box size - constraint is set at current_val +/- box_size */
  Eigen::VectorXd box_size_;
  Eigen::VectorXd constraint_merit_coeff_;

  SparseMatrix hessian_;
  Eigen::VectorXd gradient_;
  QuadExprs objective_nlp_;

  SparseMatrix constraint_matrix_;
  Eigen::VectorXd bounds_lower_;
  Eigen::VectorXd bounds_upper_;
  // This should be the center of the bounds
  Eigen::VectorXd constraint_constant_;

  /**
   * @brief Helper that updates the objective function constant and linear and quadratic coefficients of the QP Problem
   * @details Called by convexify()
   */
  void convexifyCosts();

  /**
   * @brief Helper that linearizes the constraints about the current point, storing the
   * jacobian as the constraint matrix and adding slack variables.
   * @details Called by convexify()
   */
  void linearizeConstraints();

  /**
   * @brief Helper that updates the NLP constraint bounds (top section)
   * @details Called by convexify()
   */
  void updateNLPConstraintBounds();

  /**
   * @brief Helper that updates the NLP variable bounds (middle section)
   * @details Called by convexify()
   */
  void updateNLPVariableBounds();

  /**
   * @brief Called by convexify() - helper that updates the slack variable bounds (bottom section)
   * @details A slack variable is referred to as an additional variable that has been introduced
   * to the optimization problem to turn a inequality constraint into an equality constraint.
   * Information taken from: https://en.wikipedia.org/wiki/Slack_variable
   *
   * As with the other variables in the augmented constraints, the slack variable cannot take on
   * negative values, as the simplex algorithm requires them to be positive or zero.
   *
   *   - If a slack variable associated with a constraint is zero at a particular candidate solution,
   *     the constraint is binding there, as the constraint restricts the possible changes from that point.
   *   - If a slack variable is positive at a particular candidate solution, the constraint is non-binding
   *     there, as the constraint does not restrict the possible changes from that point.
   *   - If a slack variable is negative at some point, the point is infeasible (not allowed), as it does
   *     not satisfy the constraint.
   *
   * Terminology
   *
   *   - If an inequality constraint holds with equality at the optimal point, the constraint is said to
   *     be binding, as the point cannot be varied in the direction of the constraint even though doing
   *     so would improve the value of the objective function.
   *   - If an inequality constraint holds as a strict inequality at the optimal point (that is, does
   *     not hold with equality), the constraint is said to be non-binding, as the point could be varied
   *     in the direction of the constraint, although it would not be optimal to do so. Under certain
   *     conditions, as for example in convex optimization, if a constraint is non-binding, the optimization
   *     problem would have the same solution even in the absence of that constraint.
   *   - If a constraint is not satisfied at a given point, the point is said to be infeasible.
   *
   * @example By introducing the slack variable y >= 0, the inequality Ax <= b can be converted
   * to the equation Ax + y = b.
   */
  void updateSlackVariableBounds();

  /** @brief This calculates the constant expression in the quadratic expression for the constraints */
  void updateConstraintsConstantExpression();
};

}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_TRAJOPT_QP_PROBLEM_H
