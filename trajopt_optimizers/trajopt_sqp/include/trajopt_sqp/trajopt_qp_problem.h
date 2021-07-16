#ifndef TRAJOPT_SQP_TRAJOPT_QP_PROBLEM_H
#define TRAJOPT_SQP_TRAJOPT_QP_PROBLEM_H

#include <trajopt_sqp/qp_problem.h>
#include <trajopt_sqp/utils.h>
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

  /**
   * @brief Add one individual set of variables to the optimization problem.
   * @param variable_set  The selection of optimization variables.
   *
   * This function can be called multiple times, with multiple sets, e.g.
   * one set that parameterizes a body trajectory, the other that resembles
   * the optimal timing values. This function correctly appends the
   * individual variables sets and ensures correct order of Jacobian columns.
   */
  void addVariableSet(ifopt::VariableSet::Ptr variable_set);

  /**
   * @brief Add a set of multiple constraints to the optimization problem.
   * @param constraint_set  This can be 1 to infinity number of constraints.
   *
   * This function can be called multiple times for different sets of
   * constraints. It makes sure the overall constraint and Jacobian correctly
   * considers all individual constraint sets.
   */
  void addConstraintSet(ifopt::ConstraintSet::Ptr constraint_set);

  /**
   * @brief Add a squared cost term to the problem.
   * @param constraint_set The constraint set to be evaluated as a squared cost.
   *
   * This function can be called multiple times if the constraint function is
   * composed of different cost terms. It makes sure the overall value and
   * gradient is considering each individual cost.
   */
  void addCostSet(ifopt::ConstraintSet::Ptr constraint_set, CostPenaltyType penalty_type);

  /**
   * @brief This setups the QP problems based on the constraints and cost sets added to the problem.
   * @details This must be called after all constraints and costs have been added to the problem
   */
  void setup();

  void setVariables(const double* x) override;
  Eigen::VectorXd getVariableValues() const override;

  void convexify() override;

  void convexifyCosts();

  void linearizeConstraints();
  void updateNLPConstraintBounds();
  void updateNLPVariableBounds();
  void updateSlackVariableBounds();

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

  /** @brief This calculates the constant expression in the quadratic expression for the constraints */
  void updateConstraintsConstantExpression();
};

}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_TRAJOPT_QP_PROBLEM_H
