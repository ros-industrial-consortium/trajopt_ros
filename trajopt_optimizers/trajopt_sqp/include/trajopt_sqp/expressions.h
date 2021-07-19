#ifndef TRAJOPT_SQP_EXPRESSIONS_H
#define TRAJOPT_SQP_EXPRESSIONS_H

#include <Eigen/Geometry>
#include <trajopt_sqp/types.h>

namespace trajopt_sqp
{
struct Exprs
{
  virtual ~Exprs() = default;
  virtual Eigen::VectorXd values(const Eigen::Ref<Eigen::VectorXd>& x) const = 0;
};

struct AffExprs : Exprs
{
  Eigen::VectorXd constants;
  SparseMatrix linear_coeffs;
  Eigen::VectorXd values(const Eigen::Ref<Eigen::VectorXd>& x) const override;
};

struct QuadExprs : Exprs
{
  QuadExprs() = default;
  QuadExprs(Eigen::Index num_cost, Eigen::Index num_vars);

  /**
   * @brief The constant for the equations
   * @details Each row represents a different equations constant
   */
  Eigen::VectorXd constants;

  /**
   * @brief The linear coefficients for the equations
   * @details Each row represents a different equation linear coefficients
   */
  SparseMatrix linear_coeffs;

  /**
   * @brief The quadratic coefficients for each equation (aka. row)
   * @details Each entry represents a different equation quadratic coefficients
   */
  std::vector<SparseMatrix> quadratic_coeffs;

  /**
   * @brief The objective linear coefficients
   * @details This is the sum of the equation gradient coefficients (linear_coeffs)
   */
  Eigen::VectorXd objective_linear_coeffs;

  /**
   * @brief The objective quadratic coefficients
   * @details This is the sum of the equation quadratic coefficients (quadratic_coeffs)
   */
  SparseMatrix objective_quadratic_coeffs;

  Eigen::VectorXd values(const Eigen::Ref<Eigen::VectorXd>& x) const override;
};

/**
 * @brief Given the function error and jacobian calculate the linear approximation
 * @details This was derived using the reference below, but the CostFromFunc::convex in the original trajopt is
 * different References:
 * https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/tangent-planes-and-local-linearization/a/local-linearization
 *  Where:
 *    x = x0
 *    f(x0​) = func_errors
 *    ∇f(x0​) = func_jacobian
 *
 *  Therefore:
 *    a = f(x0​) - ∇f(x0​) * x
 *    b = ∇f(x0​)
 *
 * @param func_errors The functions error
 * @param func_jacobian The functions jacobian
 * @param x The value used to calculate the error and jacobian
 * @return A linear approximation
 */
AffExprs createAffExprs(const Eigen::Ref<Eigen::VectorXd>& func_error,
                        const Eigen::Ref<const SparseMatrix>& func_jacobian,
                        const Eigen::Ref<const Eigen::VectorXd>& x);

/**
 * @brief Given the function error, jacobian and hessians calculate the quadratic approximation
 * @details This was derived using the reference below, but the CostFromFunc::convex in the original trajopt is
 * different References:
 * https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/quadratic-approximations/a/quadratic-approximation
 *  Where:
 *    x = x0
 *    f(x0​) = func_errors
 *    ∇f(x0​) = func_jacobian
 *    H = func_hessians
 *
 *  Therefore:
 *    c = 0.5 * H​(x0​);
 *    a = f(x0​) - ∇f(x0​) * x + x.transpose() * c * x
 *    b = ∇f(x0​) - (2 * c) * x
 *
 * @param func_errors The functions error
 * @param func_jacobian The functions jacobian
 * @param func_hessians The functions hessian
 * @param x The value used to calculate the error, jacobian and hessian
 * @return A quadratic approximation
 */
QuadExprs createQuadExprs(const Eigen::Ref<Eigen::VectorXd>& func_errors,
                          const Eigen::Ref<const SparseMatrix>& func_jacobian,
                          const std::vector<SparseMatrix>& func_hessians,
                          const Eigen::Ref<const Eigen::VectorXd>& x);

QuadExprs squareAffExprs(const AffExprs& aff_expr);

}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_EXPRESSIONS_H
