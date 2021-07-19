#include <trajopt_sqp/expressions.h>

namespace trajopt_sqp
{
Eigen::VectorXd AffExprs::values(const Eigen::Ref<Eigen::VectorXd>& x) const { return constants + (linear_coeffs * x); }

QuadExprs::QuadExprs(Eigen::Index num_cost, Eigen::Index num_vars)
  : constants(Eigen::VectorXd::Zero(num_cost))
  , linear_coeffs(num_cost, num_vars)
  , objective_linear_coeffs(Eigen::VectorXd::Zero(num_vars))
  , objective_quadratic_coeffs(num_vars, num_vars)
{
  quadratic_coeffs.reserve(static_cast<std::size_t>(num_cost));
}

Eigen::VectorXd QuadExprs::values(const Eigen::Ref<Eigen::VectorXd>& x) const
{
  Eigen::VectorXd result_lin = constants + (linear_coeffs * x);
  Eigen::VectorXd result_quad = result_lin;
  for (std::size_t i = 0; i < quadratic_coeffs.size(); ++i)
  {
    const auto& eq_quad_coeffs = quadratic_coeffs[i];
    if (eq_quad_coeffs.nonZeros() > 0)
      result_quad(static_cast<Eigen::Index>(i)) += (x.transpose() * eq_quad_coeffs * x);
  }
  return result_quad;
}

AffExprs createAffExprs(const Eigen::Ref<Eigen::VectorXd>& func_error,
                        const Eigen::Ref<const SparseMatrix>& func_jacobian,
                        const Eigen::Ref<const Eigen::VectorXd>& x)
{
  AffExprs aff_expr;

  aff_expr.constants = func_error - (func_jacobian * x);
  aff_expr.linear_coeffs = func_jacobian;

  return aff_expr;
}

QuadExprs createQuadExprs(const Eigen::Ref<Eigen::VectorXd>& func_errors,
                          const Eigen::Ref<const SparseMatrix>& func_jacobian,
                          const std::vector<SparseMatrix>& func_hessians,
                          const Eigen::Ref<const Eigen::VectorXd>& x)
{
  QuadExprs quad_exprs;

  quad_exprs.quadratic_coeffs.resize(static_cast<std::size_t>(func_errors.rows()));
  quad_exprs.linear_coeffs.resize(func_jacobian.rows(), func_jacobian.cols());
  quad_exprs.constants = func_errors - (func_jacobian * x);

  for (Eigen::Index i = 0; i < func_errors.rows(); ++i)
  {
    const auto& eq_hessian = func_hessians[static_cast<std::size_t>(i)];
    if (eq_hessian.nonZeros() > 0)
    {
      auto& c = quad_exprs.quadratic_coeffs[static_cast<std::size_t>(i)];
      c = 0.5 * eq_hessian;

      quad_exprs.constants(i) += (x.transpose() * c * x);
      quad_exprs.linear_coeffs.row(i) = func_jacobian.row(i) - ((2.0 * c) * x).transpose();
    }
  }

  return quad_exprs;
}

QuadExprs squareAffExprs(const AffExprs& aff_expr)
{
  QuadExprs quad_expr;
  quad_expr.constants = aff_expr.constants.array().square();
  quad_expr.linear_coeffs = (2 * aff_expr.constants).asDiagonal() * aff_expr.linear_coeffs;
  quad_expr.quadratic_coeffs.resize(static_cast<std::size_t>(aff_expr.constants.rows()));
  quad_expr.objective_linear_coeffs = Eigen::VectorXd::Zero(aff_expr.linear_coeffs.cols());
  quad_expr.objective_quadratic_coeffs.resize(aff_expr.linear_coeffs.cols(), aff_expr.linear_coeffs.cols());

  for (Eigen::Index i = 0; i < aff_expr.constants.rows(); ++i)
  {
    // Increment the objective linear coefficients
    quad_expr.objective_linear_coeffs += quad_expr.linear_coeffs.row(i);

    // Now calculate the quadratic coefficients
    auto eq_affexpr_coeffs = aff_expr.linear_coeffs.row(i);
    SparseMatrix eq_quadexpr_coeffs = eq_affexpr_coeffs.transpose() * eq_affexpr_coeffs;

    // Store the quadtratic coeffs and increment the objective quadratic coefficients
    if (eq_quadexpr_coeffs.nonZeros() > 0)
    {
      quad_expr.quadratic_coeffs[static_cast<std::size_t>(i)] = eq_quadexpr_coeffs;
      quad_expr.objective_quadratic_coeffs += eq_quadexpr_coeffs;
    }
  }

  return quad_expr;
}
}  // namespace trajopt_sqp
