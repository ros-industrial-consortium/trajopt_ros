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
                        const Eigen::Ref<const SparseMatrix>& func_gradient,
                        const Eigen::Ref<const Eigen::VectorXd>& x)
{
  AffExprs aff_expr;

  aff_expr.constants = func_error - (func_gradient * x);
  aff_expr.linear_coeffs = func_gradient;

  return aff_expr;
}

QuadExprs createQuadExprs(const Eigen::Ref<Eigen::VectorXd>& /*func_errors*/,
                          const Eigen::Ref<const SparseMatrix>& /*func_gradients*/,
                          const std::vector<SparseMatrix>& /*func_hessians*/,
                          const Eigen::Ref<const Eigen::VectorXd>& /*x*/)
{
  throw std::runtime_error("Need to fix compiler errors");
  //  QuadExprs quad_expr;

  //  quad_expr.constants = func_errors - (func_gradients * x);
  //  quad_expr.linear_coeffs = func_gradients;
  //  for (Eigen::Index i = 0; i < func_errors.rows(); ++i)
  //  {
  //    const auto& eq_hessian = func_hessians[static_cast<std::size_t>(i)];
  //    if (eq_hessian.nonZeros() > 0)
  //    {
  //      Eigen::MatrixXd pos_hess_dense = Eigen::MatrixXd::Zero(x.size(), x.size());
  //      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(eq_hessian);
  //      Eigen::VectorXd eigvals = es.eigenvalues();
  //      Eigen::MatrixXd eigvecs = es.eigenvectors();
  //      for (long int i = 0, end = x.size(); i != end; ++i)
  //      {  // tricky --- eigen size() is signed
  //        if (eigvals(i) > 0)
  //          pos_hess_dense += (eigvals(i) * eigvecs.col(i) * eigvecs.col(i).transpose());
  //      }

  //      auto temp = (eq_hessian * x);
  //      quad_expr.constants(i) += 0.5 * x.dot(temp);
  //      quad_expr.linear_coeffs.row(i) -= temp.transpose();
  //      quad_expr.quadratic_coeffs[static_cast<std::size_t>(i)].resize(x.rows(), x.rows());

  //      SparseMatrix pos_hess = pos_hess_dense.sparseView(0, 1e-7);
  //      if (pos_hess.nonZeros() > 0)
  //      {
  //        std::vector<Eigen::Triplet<double>> quadratic_coeffs_triplet_list;
  //        quadratic_coeffs_triplet_list.reserve(static_cast<std::size_t>(pos_hess.nonZeros()) * 3);
  //        // Add jacobian to triplet list
  //        for (int k = 0; k < pos_hess.outerSize(); ++k)
  //        {
  //          for (SparseMatrix::InnerIterator it(pos_hess, k); it; ++it)
  //          {
  //            if (it.row() == it.col())
  //              quadratic_coeffs_triplet_list.emplace_back(it.row(), it.col(), it.value());
  //            else
  //              quadratic_coeffs_triplet_list.emplace_back(it.row(), it.col(), 0.5 * it.value());
  //          }
  //        }
  //        quad_expr.quadratic_coeffs[static_cast<std::size_t>(i)].setFromTriplets(quadratic_coeffs_triplet_list.begin(),
  //        quadratic_coeffs_triplet_list.end());
  //      }
  //    }
  //  }

  //  return quad_expr;
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

    // Fill squared costs
    // Create triplet list of nonzero hessian
    if (eq_quadexpr_coeffs.nonZeros() > 0)
    {
      std::vector<Eigen::Triplet<double>> eq_hessian_triplet_list;
      eq_hessian_triplet_list.reserve(static_cast<std::size_t>(2 * aff_expr.linear_coeffs.cols()) * 3);
      // Add jacobian to triplet list
      for (int k = 0; k < eq_quadexpr_coeffs.outerSize(); ++k)
      {
        for (SparseMatrix::InnerIterator it(eq_quadexpr_coeffs, k); it; ++it)
        {
          eq_hessian_triplet_list.emplace_back(it.row(), it.col(), it.value());
        }
      }
      auto& cost_quadexpr_hessian = quad_expr.quadratic_coeffs[static_cast<std::size_t>(i)];
      cost_quadexpr_hessian.resize(aff_expr.linear_coeffs.cols(), aff_expr.linear_coeffs.cols());
      cost_quadexpr_hessian.setFromTriplets(eq_hessian_triplet_list.begin(), eq_hessian_triplet_list.end());
      quad_expr.objective_quadratic_coeffs += cost_quadexpr_hessian;

      assert(cost_quadexpr_hessian.nonZeros() != cost_quadexpr_hessian.size());
    }
  }

  assert(quad_expr.linear_coeffs.nonZeros() != quad_expr.linear_coeffs.size());

  return quad_expr;
}
}  // namespace trajopt_sqp
