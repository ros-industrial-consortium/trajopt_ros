
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <iostream>

namespace trajopt_sqp
{
TrajOptQPProblem::TrajOptQPProblem()
  : constraints_("constraint-sets", false)
  , squared_costs_("squared-cost-terms", false)
  , abs_costs_("abs-cost-terms", false)
  , hing_costs_("hing-cost-terms", false)
{
  variables_ = std::make_shared<ifopt::Composite>("variable-sets", false);
}

void TrajOptQPProblem::addVariableSet(ifopt::VariableSet::Ptr variable_set)
{
  variables_->AddComponent(variable_set);
  initialized_ = false;
}

void TrajOptQPProblem::addConstraintSet(ifopt::ConstraintSet::Ptr constraint_set)
{
  constraint_set->LinkWithVariables(variables_);
  constraints_.AddComponent(constraint_set);
  initialized_ = false;
}

void TrajOptQPProblem::addCostSet(ifopt::ConstraintSet::Ptr constraint_set, CostPenaltyType penalty_type)
{
  constraint_set->LinkWithVariables(variables_);
  switch (penalty_type)
  {
    case CostPenaltyType::SQUARED:
    {
      squared_costs_.AddComponent(constraint_set);
      break;
    }
    case CostPenaltyType::ABSOLUTE:
    {
      abs_costs_.AddComponent(constraint_set);
      break;
    }
      //    case CostPenaltyType::HING:
      //    {
      //      hing_costs_.AddComponent(constraint_set);
      //      break;
      //    }
    default:
    {
      throw std::runtime_error("Unsupport CostPenaltyType!");
      break;
    }
  }

  initialized_ = false;
}

void TrajOptQPProblem::setup()
{
  num_qp_vars_ = getNumNLPVars();
  num_qp_cnts_ = getNumNLPConstraints() + getNumNLPVars();
  box_size_ = Eigen::VectorXd::Constant(getNumNLPVars(), 1e-1);
  constraint_merit_coeff_ = Eigen::VectorXd::Constant(getNumNLPConstraints(), 10);

  // Get NLP Cost and Constraint Names for Debug Print
  for (const auto& cnt : constraints_.GetComponents())
  {
    // Loop over each constraint in the set
    for (Eigen::Index j = 0; j < cnt->GetRows(); j++)
      constraint_names_.push_back(cnt->GetName() + "_" + std::to_string(j));
  }

  for (const auto& cost : squared_costs_.GetComponents())
  {
    for (Eigen::Index j = 0; j < cost->GetRows(); j++)
      cost_names_.push_back(cost->GetName() + "_" + std::to_string(j));
  }

  for (const auto& cost : abs_costs_.GetComponents())
  {
    for (Eigen::Index j = 0; j < cost->GetRows(); j++)
      cost_names_.push_back(cost->GetName() + "_" + std::to_string(j));
  }

  for (const auto& cost : hing_costs_.GetComponents())
  {
    for (Eigen::Index j = 0; j < cost->GetRows(); j++)
      cost_names_.push_back(cost->GetName() + "_" + std::to_string(j));
  }

  ////////////////////////////////////////////////////////
  // Get NLP bounds and detect constraint type
  ////////////////////////////////////////////////////////
  Eigen::VectorXd nlp_bounds_l(getNumNLPConstraints());
  Eigen::VectorXd nlp_bounds_u(getNumNLPConstraints());
  // Convert constraint bounds to VectorXd
  std::vector<ifopt::Bounds> cnt_bounds = constraints_.GetBounds();
  for (Eigen::Index i = 0; i < getNumNLPConstraints(); i++)
  {
    nlp_bounds_l[i] = cnt_bounds[static_cast<std::size_t>(i)].lower_;
    nlp_bounds_u[i] = cnt_bounds[static_cast<std::size_t>(i)].upper_;
  }

  // Detect constraint type
  Eigen::VectorXd nlp_bounds_diff = nlp_bounds_u - nlp_bounds_l;
  constraint_types_.resize(static_cast<std::size_t>(getNumNLPConstraints()));
  for (std::size_t i = 0; i < static_cast<std::size_t>(nlp_bounds_diff.size()); i++)
  {
    if (std::abs(nlp_bounds_diff[static_cast<Eigen::Index>(i)]) < 1e-3)
    {
      constraint_types_[i] = ConstraintType::EQ;
      // Add 2 slack variables for L1 loss
      num_qp_vars_ += 2;
      num_qp_cnts_ += 2;
    }
    else
    {
      constraint_types_[i] = ConstraintType::INEQ;
      // Add 1 slack variable for hinge loss
      num_qp_vars_ += 1;
      num_qp_cnts_ += 1;
    }
  }

  // Initialize the constraint bounds
  bounds_lower_ = Eigen::VectorXd::Constant(num_qp_cnts_, -double(INFINITY));
  bounds_upper_ = Eigen::VectorXd::Constant(num_qp_cnts_, double(INFINITY));

  // Set initialized
  initialized_ = true;
}

void TrajOptQPProblem::setVariables(const double* x)
{
  variables_->SetVariables(Eigen::Map<const Eigen::VectorXd>(x, variables_->GetRows()));
}

Eigen::VectorXd TrajOptQPProblem::getVariableValues() const { return variables_->GetValues(); }

void TrajOptQPProblem::convexify()
{
  assert(initialized_);

  // This must be called prior to updateGradient
  convexifyCosts();

  linearizeConstraints();

  // The three above must be called before rest to update internal data

  updateConstraintsConstantExpression();

  updateNLPConstraintBounds();

  updateNLPVariableBounds();

  updateSlackVariableBounds();
}

void TrajOptQPProblem::convexifyCosts()
{
  ////////////////////////////////////////////////////////
  // Set the Hessian (empty for now)
  ////////////////////////////////////////////////////////
  hessian_.resize(num_qp_vars_, num_qp_vars_);
  hessian_nlp_.resize(getNumNLPVars(), getNumNLPVars());
  hessian_nlp_vec_.resize(static_cast<std::size_t>(getNumNLPCosts()));

  /**
   * @note See CostFromFunc::convex in modeling_utils.cpp.
   * This should be multiplied by 0.5 when implemented
   * hessian_ = 0.5 * nlp_->GetHessianOfCosts();
   */

  ////////////////////////////////////////////////////////
  // Set the gradient of the NLP costs
  ////////////////////////////////////////////////////////
  gradient_ = Eigen::VectorXd::Zero(num_qp_vars_);
  gradient_nlp_.resize(getNumNLPCosts(), getNumNLPVars());

  Eigen::VectorXd x_initial = variables_->GetValues().head(getNumNLPVars());
  Eigen::VectorXd cost_initial_value(getNumNLPCosts());

  // Create triplet list of nonzero gradients
  std::vector<Eigen::Triplet<double>> grad_triplet_list;
  grad_triplet_list.reserve(static_cast<std::size_t>(getNumNLPVars() * getNumNLPCosts()) * 3);

  // Create triplet list of nonzero hessian
  std::vector<Eigen::Triplet<double>> hessian_triplet_list;
  hessian_triplet_list.reserve(static_cast<std::size_t>(getNumNLPVars() * getNumNLPCosts() * getNumNLPCosts()) * 3);

  // Process Squared Costs
  Eigen::Index grad_start_index{ 0 };
  if (squared_costs_.GetRows() > 0)
  {
    ifopt::ConstraintSet::Jacobian cnt_jac = squared_costs_.GetJacobian();
    ifopt::ConstraintSet::VectorXd cnt_vals = squared_costs_.GetValues();

    Eigen::VectorXd cost_error = trajopt_ifopt::calcBoundsErrors(cnt_vals, squared_costs_.GetBounds());
    Eigen::VectorXd cost_affexpr_constant = cost_error - (cnt_jac * x_initial);
    ifopt::ConstraintSet::Jacobian cost_affine_coeff = cnt_jac;

    Eigen::VectorXd cost_quadexpr_constant = cost_affexpr_constant.array().square();

    // store cost constant
    cost_initial_value.topRows(squared_costs_.GetRows()) = cost_quadexpr_constant;

    ifopt::ConstraintSet::Jacobian cost_quadexpr_affexpr_coeffs =
        (2 * cost_affexpr_constant).asDiagonal() * cost_affine_coeff;

    // Fill squared costs
    if (cost_quadexpr_affexpr_coeffs.nonZeros() > 0)
    {
      // Add jacobian to triplet list
      for (int k = 0; k < cost_quadexpr_affexpr_coeffs.outerSize(); ++k)
      {
        for (ifopt::ConstraintSet::Jacobian::InnerIterator it(cost_quadexpr_affexpr_coeffs, k); it; ++it)
        {
          grad_triplet_list.emplace_back(grad_start_index + it.row(), it.col(), it.value());

          // The row and column is flipped because gradient_ is a vector
          gradient_(it.col()) += it.value();
        }
      }
    }

    for (Eigen::Index i = 0; i < cnt_vals.rows(); ++i)
    {
      auto eq_affexpr_coeffs = cost_affine_coeff.row(i);
      ifopt::ConstraintSet::Jacobian eq_quadexpr_coeffs = eq_affexpr_coeffs.transpose() * eq_affexpr_coeffs;

      // Fill squared costs
      // Create triplet list of nonzero hessian
      std::vector<Eigen::Triplet<double>> eq_hessian_triplet_list;
      if (eq_quadexpr_coeffs.nonZeros() > 0)
      {
        eq_hessian_triplet_list.reserve(static_cast<std::size_t>(getNumNLPVars() * getNumNLPVars()) * 3);
        // Add jacobian to triplet list
        for (int k = 0; k < eq_quadexpr_coeffs.outerSize(); ++k)
        {
          for (ifopt::ConstraintSet::Jacobian::InnerIterator it(eq_quadexpr_coeffs, k); it; ++it)
          {
            eq_hessian_triplet_list.emplace_back(it.row(), it.col(), it.value());
          }
        }
        auto& cost_quadexpr_hessian = hessian_nlp_vec_[static_cast<std::size_t>(i)];
        cost_quadexpr_hessian.resize(getNumNLPVars(), getNumNLPVars());
        cost_quadexpr_hessian.setFromTriplets(eq_hessian_triplet_list.begin(), eq_hessian_triplet_list.end());
        hessian_nlp_ += cost_quadexpr_hessian;
      }
    }

    grad_start_index += cnt_vals.rows();
  }

  // Process ABS Costs
  if (abs_costs_.GetRows() > 0)
  {
    throw std::runtime_error("Absolute cost is not implemented!");
    //    ifopt::ConstraintSet::Jacobian cnt_jac = abs_costs_.GetJacobian();
    //    ifopt::ConstraintSet::VectorXd cnt_vals = abs_costs_.GetValues();

    //    Eigen::VectorXd abs_cnt_error = trajopt_ifopt::calcBoundsErrors(cnt_vals, abs_costs_.GetBounds());
    //    Eigen::VectorXd abs_cost_error = abs_cnt_error;
    //    abs_cost_error.cwiseAbs();
    //    Eigen::VectorXd coeff = abs_cnt_error.array() / abs_cost_error.array();

    //    // Fill abs costs
    //    ifopt::ConstraintSet::Jacobian abs_nlp_jac = 2 * coeff.asDiagonal() * cnt_jac;
    //    if (abs_nlp_jac.nonZeros() > 0)
    //    {
    //      // Add jacobian to triplet list
    //      for (int k = 0; k < abs_nlp_jac.outerSize(); ++k)
    //      {
    //        for (ifopt::ConstraintSet::Jacobian::InnerIterator it(abs_nlp_jac, k); it; ++it)
    //        {
    //          tripletList.emplace_back(start_index+ it.row(), it.col(), it.value());
    //        }
    //      }
    //    }

    //    start_index += cnt_vals.rows();

    //    // Fill abs costs
    //    ifopt::ConstraintSet::Jacobian abs_cost_jac = coeff.sparseView() * cnt_jac;
    //    if (abs_cost_jac.nonZeros() > 0)
    //      gradient_.topRows(getNumNLPVars()) = gradient_.topRows(getNumNLPVars()) +
    //      abs_cost_jac.toDense().transpose();
  }

  if (hing_costs_.GetRows() > 0)
  {
    throw std::runtime_error("Absolute cost is not implemented!");
    //    ifopt::ConstraintSet::Jacobian cnt_jac = hing_costs_.GetJacobian();
    //    ifopt::ConstraintSet::VectorXd cnt_vals = hing_costs_.GetValues();

    //    Eigen::VectorXd cost_error = trajopt_ifopt::calcBoundsErrors(cnt_vals, hing_costs_.GetBounds());
    //    Eigen::VectorXd cost_affexpr_constant = cost_error - (cnt_jac * x_initial);
    //    ifopt::ConstraintSet::Jacobian cost_affine_coeff = cnt_jac;

    //    Eigen::VectorXd cost_quadexpr_constant = cost_affexpr_constant;
    //    cost_quadexpr_constant.array().pow(2);

    //    ifopt::ConstraintSet::Jacobian cost_quadexpr_affexpr_coeffs = (2 * cost_affexpr_constant).asDiagonal() *
    //    cost_affine_coeff;

    //    // Fill squared costs
    //    if (cost_quadexpr_affexpr_coeffs.nonZeros() > 0)
    //    {
    //      // Add jacobian to triplet list
    //      for (int k = 0; k < cost_quadexpr_affexpr_coeffs.outerSize(); ++k)
    //      {
    //        for (ifopt::ConstraintSet::Jacobian::InnerIterator it(cost_quadexpr_affexpr_coeffs, k); it; ++it)
    //        {
    //          grad_triplet_list.emplace_back(grad_start_index + it.row(), it.col(), it.value());
    //        }
    //      }
    //    }

    //    for (Eigen::Index i = 0; i < cnt_vals.rows(); ++i)
    //    {

    //      auto eq_affexpr_coeffs = cost_quadexpr_affexpr_coeffs.row(i);
    //      ifopt::ConstraintSet::Jacobian eq_quadexpr_coeffs = eq_affexpr_coeffs * eq_affexpr_coeffs.transpose();

    //      // Fill squared costs
    //      // Create triplet list of nonzero hessian
    //      std::vector<Eigen::Triplet<double>> eq_hessian_triplet_list;
    //      if (eq_quadexpr_coeffs.nonZeros() > 0)
    //      {
    //        eq_hessian_triplet_list.reserve(static_cast<std::size_t>(getNumNLPVars() * getNumNLPVars()) * 3);
    //        // Add jacobian to triplet list
    //        for (int k = 0; k < eq_quadexpr_coeffs.outerSize(); ++k)
    //        {
    //          for (ifopt::ConstraintSet::Jacobian::InnerIterator it(eq_quadexpr_coeffs, k); it; ++it)
    //          {
    //            eq_hessian_triplet_list.emplace_back(it.row(), it.col(), it.value());
    //          }
    //        }
    //        hessian_nlp[i].setFromTriplets(eq_hessian_triplet_list.begin(), eq_hessian_triplet_list.end());
    //        hessian_nlp_ += hessian_nlp[i];
    //      }

    //      hessian_start_index += getNumNLPVars();
    //    }

    //    grad_start_index += cnt_vals.rows();

    //    // Fill squared costs
    //    ifopt::ConstraintSet::Jacobian squared_cost_jac = 2 * squared_cost_error.transpose().sparseView() * cnt_jac;
    //    if (squared_cost_jac.nonZeros() > 0)
    //      gradient_.topRows(getNumNLPVars()) = gradient_.topRows(getNumNLPVars()) +
    //      squared_cost_jac.toDense().transpose();
  }

  gradient_nlp_.setFromTriplets(grad_triplet_list.begin(), grad_triplet_list.end());
  cost_constant_ = cost_initial_value;

  if (hessian_nlp_.nonZeros() > 0)
  {
    hessian_.reserve(hessian_nlp_.nonZeros());
    for (int k = 0; k < hessian_nlp_.outerSize(); ++k)
    {
      for (ifopt::ConstraintSet::Jacobian::InnerIterator it(hessian_nlp_, k); it; ++it)
        hessian_.coeffRef(it.row(), it.col()) += it.value();
    }
  }

  /**
   * @note See CostFromFunc::convex in modeling_utils.cpp. Once Hessian has been implemented
   *
   * if (!full_hessian_) // Does not contain bilinear terms, eg. x1 * x2, only x1^2 and x2^2
   * {
   *   double val;
   *   Eigen::VectorXd grad, hess;
   *   calcGradAndDiagHess(*f_, x_eigen, epsilon_, val, grad, hess);
   *   quad = a + b*x + c*x^2
   *   a = val - grad.dot(x_eigen) + .5 * x_eigen.dot(hess.cwiseProduct(x_eigen));
   *   b = grad - hess.cwiseProduct(x_eigen)
   *   c = 0.5 * hess
   * }
   * else // contains bilinear terms
   * {
   *   double val;
   *   Eigen::VectorXd grad;
   *   Eigen::MatrixXd hess;
   *   calcGradHess(f_, x_eigen, epsilon_, val, grad, hess);
   *
   *   Eigen::MatrixXd pos_hess = Eigen::MatrixXd::Zero(x_eigen.size(), x_eigen.size());
   *   Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(hess);
   *   Eigen::VectorXd eigvals = es.eigenvalues();
   *   Eigen::MatrixXd eigvecs = es.eigenvectors();
   *   for (long int i = 0, end = x_eigen.size(); i != end; ++i)
   *   {  // tricky --- eigen size() is signed
   *     if (eigvals(i) > 0)
   *       pos_hess += eigvals(i) * eigvecs.col(i) * eigvecs.col(i).transpose();
   *   }
   *   quad = a + b*x + c*x^2 +
   *   a = val - grad.dot(x_eigen) + .5 * x_eigen.dot(pos_hess * x_eigen);
   *   b = grad - pos_hess * x_eigen
   *   for (long int i = 0, end = x_eigen.size(); i != end; ++i)
   *   {  // tricky --- eigen size() is signed
   *     c(i,i) = (pos_hess(i, i) / 2);
   *     for (long int j = i + 1; j != end; ++j)
   *     {  // tricky --- eigen size() is signed
   *       c(i, j) = pos_hess(i, j);
   *     }
   *   }
   * }
   */

  ////////////////////////////////////////////////////////
  // Set the gradient of the constraint slack variables
  ////////////////////////////////////////////////////////
  {
    Eigen::Index current_var_index = getNumNLPVars();
    for (Eigen::Index i = 0; i < getNumNLPConstraints(); i++)
    {
      if (constraint_types_[static_cast<std::size_t>(i)] == ConstraintType::EQ)
      {
        gradient_[current_var_index] = constraint_merit_coeff_[i];
        gradient_[current_var_index + 1] = constraint_merit_coeff_[i];

        current_var_index += 2;
      }
      else
      {
        gradient_[current_var_index] = constraint_merit_coeff_[i];
        current_var_index++;
      }
    }
  }
}

void TrajOptQPProblem::linearizeConstraints()
{
  Eigen::SparseMatrix<double> jac = constraints_.GetJacobian();

  // Create triplet list of nonzero constraints
  using T = Eigen::Triplet<double>;
  std::vector<T> tripletList;
  tripletList.reserve(static_cast<std::size_t>(jac.nonZeros() + num_qp_vars_) * 3);

  // Add jacobian to triplet list
  for (int k = 0; k < jac.outerSize(); ++k)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jac, k); it; ++it)
    {
      tripletList.emplace_back(it.row(), it.col(), it.value());
    }
  }

  // Add the slack variables to each constraint
  Eigen::Index current_column_index = getNumNLPVars();
  for (Eigen::Index i = 0; i < getNumNLPConstraints(); i++)
  {
    if (constraint_types_[static_cast<std::size_t>(i)] == ConstraintType::EQ)
    {
      tripletList.emplace_back(i, current_column_index, 1);
      tripletList.emplace_back(i, current_column_index + 1, -1);
      current_column_index += 2;
    }
    else
    {
      tripletList.emplace_back(i, current_column_index, -1);
      current_column_index++;
    }
  }

  // Add a diagonal matrix for the variable limits (including slack variables since the merit coeff is only applied in
  // the cost) below the actual constraints
  for (Eigen::Index i = 0; i < num_qp_vars_; i++)
    tripletList.emplace_back(i + jac.rows(), i, 1);

  // Insert the triplet list into the sparse matrix
  constraint_matrix_.resize(num_qp_cnts_, num_qp_vars_);
  constraint_matrix_.reserve(jac.nonZeros() + num_qp_vars_);
  constraint_matrix_.setFromTriplets(tripletList.begin(), tripletList.end());
}

void TrajOptQPProblem::updateConstraintsConstantExpression()
{
  if (getNumNLPConstraints() == 0)
    return;

  // Get values about which we will linearize
  Eigen::VectorXd x_initial = variables_->GetValues().head(getNumNLPVars());
  Eigen::VectorXd cnt_initial_value = constraints_.GetValues();

  // In the case of a QP problem the costs and constraints are represented as
  // quadratic functions is f(x) = a + b * x + c * x^2.
  // Currently for constraints we do not leverage the Hessian so the quadratic
  // function is f(x) = a + b * x
  // When convexifying the function it need to produce the same constraint values at the values used to calculate
  // the jacobian, so f(x_initial) = a + b * x = cnt_initial_value.
  // Therefore a = cnt_initial_value - b * x
  //     where: b = jac (calculated below)
  //            x = x_initial
  //            a = quadratic constant (constraint_constant_)
  //
  // Note: This is not used by the QP solver directly but by the Trust Regions Solver
  //       to calculate the merit of the solve.

  // The block excludes the slack variables
  Eigen::SparseMatrix<double> jac = constraint_matrix_.block(0, 0, getNumNLPConstraints(), getNumNLPVars());
  constraint_constant_ = (cnt_initial_value - jac * x_initial);
}

void TrajOptQPProblem::updateNLPConstraintBounds()
{
  if (getNumNLPConstraints() == 0)
    return;

  Eigen::VectorXd cnt_bound_lower(getNumNLPConstraints());
  Eigen::VectorXd cnt_bound_upper(getNumNLPConstraints());

  // Convert constraint bounds to VectorXd
  std::vector<ifopt::Bounds> cnt_bounds = constraints_.GetBounds();
  for (Eigen::Index i = 0; i < getNumNLPConstraints(); i++)
  {
    cnt_bound_lower[i] = cnt_bounds[static_cast<std::size_t>(i)].lower_;
    cnt_bound_upper[i] = cnt_bounds[static_cast<std::size_t>(i)].upper_;
  }

  Eigen::VectorXd linearized_cnt_lower = cnt_bound_lower - constraint_constant_;
  Eigen::VectorXd linearized_cnt_upper = cnt_bound_upper - constraint_constant_;

  // Insert linearized constraint bounds
  bounds_lower_.topRows(getNumNLPConstraints()) = linearized_cnt_lower;
  bounds_upper_.topRows(getNumNLPConstraints()) = linearized_cnt_upper;
}

void TrajOptQPProblem::updateNLPVariableBounds()
{
  // This is eqivalent to BasicTrustRegionSQP::setTrustBoxConstraints
  Eigen::VectorXd x_initial = variables_->GetValues();

  // Calculate box constraints
  Eigen::VectorXd lower_box_cnt = x_initial - box_size_;
  Eigen::VectorXd upper_box_cnt = x_initial + box_size_;

  // Set the variable limits once
  std::vector<ifopt::Bounds> var_bounds = variables_->GetBounds();
  Eigen::VectorXd var_bounds_lower(getNumNLPVars());
  Eigen::VectorXd var_bounds_upper(getNumNLPVars());
  for (Eigen::Index i = 0; i < getNumNLPVars(); i++)
  {
    var_bounds_lower[i] = var_bounds[static_cast<std::size_t>(i)].lower_;
    var_bounds_upper[i] = var_bounds[static_cast<std::size_t>(i)].upper_;
  }

  // Apply box constraints and variable limits
  Eigen::VectorXd var_bounds_lower_final = var_bounds_lower.cwiseMax(lower_box_cnt);
  // Add the extra check here that the upper is bigger than the lower. It seems that there can be issues when the
  // numbers get close to 0.
  Eigen::VectorXd var_bounds_upper_final = var_bounds_upper.cwiseMin(upper_box_cnt).cwiseMax(var_bounds_lower);
  bounds_lower_.block(getNumNLPConstraints(), 0, var_bounds_lower_final.size(), 1) = var_bounds_lower_final;
  bounds_upper_.block(getNumNLPConstraints(), 0, var_bounds_upper_final.size(), 1) = var_bounds_upper_final;
}

void TrajOptQPProblem::updateSlackVariableBounds()
{
  Eigen::Index current_cnt_index = getNumNLPConstraints() + getNumNLPVars();
  for (Eigen::Index i = 0; i < getNumNLPConstraints(); i++)
  {
    if (constraint_types_[static_cast<std::size_t>(i)] == ConstraintType::EQ)
    {
      bounds_lower_[current_cnt_index] = 0;
      bounds_upper_[current_cnt_index] = double(INFINITY);
      bounds_lower_[current_cnt_index + 1] = 0;
      bounds_upper_[current_cnt_index + 1] = double(INFINITY);

      current_cnt_index += 2;
    }
    else
    {
      bounds_lower_[current_cnt_index] = 0;
      bounds_upper_[current_cnt_index] = double(INFINITY);

      current_cnt_index++;
    }
  }
}

double TrajOptQPProblem::evaluateTotalConvexCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  return evaluateConvexCosts(var_vals).sum();
}

Eigen::VectorXd TrajOptQPProblem::evaluateConvexCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  if (getNumNLPCosts() == 0)
    return Eigen::VectorXd();

  Eigen::VectorXd var_block = var_vals.head(getNumNLPVars());
  //  Eigen::VectorXd result_quad =
  //      var_block.transpose() * hessian_.block(0, 0, getNumNLPVars(), getNumNLPVars()) * var_block;
  ////  Eigen::VectorXd result_lin = var_block.transpose() * gradient_.block(0, 0, getNumNLPVars(), getNumNLPCosts());

  Eigen::VectorXd result_lin = cost_constant_ + (gradient_nlp_ * var_block);
  Eigen::VectorXd result_quad = result_lin;
  for (Eigen::Index i = 0; i < getNumNLPCosts(); ++i)
  {
    const auto& cost_quadexpr_hessian = hessian_nlp_vec_[static_cast<std::size_t>(i)];
    if (cost_quadexpr_hessian.nonZeros() > 0)
      result_quad(i) = result_lin(i) + (var_block.transpose() * cost_quadexpr_hessian * var_block);
  }
  return result_quad;
}

double TrajOptQPProblem::evaluateTotalExactCost(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  if (getNumNLPCosts() == 0)
    return 0;

  double g{ 0 };
  setVariables(var_vals.data());

  if (squared_costs_.GetRows() > 0)
  {
    Eigen::VectorXd error = trajopt_ifopt::calcBoundsViolations(squared_costs_.GetValues(), squared_costs_.GetBounds());
    g += error.squaredNorm();
  }

  if (abs_costs_.GetRows() > 0)
  {
    Eigen::VectorXd error = trajopt_ifopt::calcBoundsViolations(abs_costs_.GetValues(), abs_costs_.GetBounds());
    g += error.sum();
  }

  return g;
}

Eigen::VectorXd TrajOptQPProblem::evaluateExactCosts(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  if (getNumNLPCosts() == 0)
    return Eigen::VectorXd();

  setVariables(var_vals.data());

  Eigen::VectorXd g(getNumNLPCosts());
  Eigen::Index start_index = 0;
  if (squared_costs_.GetRows() > 0)
  {
    g.topRows(squared_costs_.GetRows()) = squared_costs_.GetValues().cwiseAbs2();
    start_index = squared_costs_.GetRows();
  }

  if (abs_costs_.GetRows() > 0)
  {
    g.middleRows(start_index, abs_costs_.GetRows()) = abs_costs_.GetValues().cwiseAbs();
    start_index = abs_costs_.GetRows();
  }

  return g;
}

Eigen::VectorXd TrajOptQPProblem::getExactCosts() { return evaluateExactCosts(variables_->GetValues()); }

Eigen::VectorXd TrajOptQPProblem::evaluateConvexConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  Eigen::VectorXd result_lin =
      constraint_matrix_.block(0, 0, getNumNLPConstraints(), getNumNLPVars()) * var_vals.head(getNumNLPVars());
  Eigen::VectorXd constraint_value = constraint_constant_ + result_lin;
  return trajopt_ifopt::calcBoundsViolations(constraint_value, constraints_.GetBounds());
}

Eigen::VectorXd TrajOptQPProblem::evaluateExactConstraintViolations(const Eigen::Ref<const Eigen::VectorXd>& var_vals)
{
  setVariables(var_vals.data());
  Eigen::VectorXd cnt_vals = constraints_.GetValues();
  return trajopt_ifopt::calcBoundsViolations(cnt_vals, constraints_.GetBounds());
}

Eigen::VectorXd TrajOptQPProblem::getExactConstraintViolations()
{
  return evaluateExactConstraintViolations(variables_->GetValues());
}

void TrajOptQPProblem::scaleBoxSize(double& scale)
{
  box_size_ = box_size_ * scale;
  updateNLPVariableBounds();
}

void TrajOptQPProblem::setBoxSize(const Eigen::Ref<const Eigen::VectorXd>& box_size)
{
  assert(box_size.size() == getNumNLPVars());
  box_size_ = box_size;
  updateNLPVariableBounds();
}

Eigen::VectorXd TrajOptQPProblem::getBoxSize() const { return box_size_; }

void TrajOptQPProblem::print() const
{
  Eigen::IOFormat format(3);

  std::cout << "-------------- QPProblem::print() --------------" << std::endl;
  std::cout << "Num NLP Vars: " << getNumNLPVars() << std::endl;
  std::cout << "Num QP Vars: " << num_qp_vars_ << std::endl;
  std::cout << "Num NLP Constraints: " << num_qp_cnts_ << std::endl;
  std::cout << "Detected Constraint Type: ";
  for (const auto& cnt : constraint_types_)
    std::cout << static_cast<int>(cnt) << ", ";

  std::cout << std::endl;
  std::cout << "box_size_: " << box_size_.transpose().format(format) << std::endl;
  std::cout << "constraint_merit_coeff_: " << constraint_merit_coeff_.transpose().format(format) << std::endl;

  std::cout << "Hessian:\n" << hessian_.toDense().format(format) << std::endl;
  std::cout << "Gradient: " << gradient_.transpose().format(format) << std::endl;
  std::cout << "Constraint Matrix:\n" << constraint_matrix_.toDense().format(format) << std::endl;
  std::cout << "bounds_lower: " << bounds_lower_.transpose().format(format) << std::endl;
  std::cout << "bounds_upper: " << bounds_upper_.transpose().format(format) << std::endl;
  std::cout << "NLP values: " << variables_->GetValues().transpose().format(format) << std::endl;
}

Eigen::Index TrajOptQPProblem::getNumNLPVars() const { return variables_->GetRows(); }
Eigen::Index TrajOptQPProblem::getNumNLPConstraints() const
{
  return static_cast<Eigen::Index>(constraints_.GetBounds().size());
}
Eigen::Index TrajOptQPProblem::getNumNLPCosts() const
{
  return (squared_costs_.GetRows() + abs_costs_.GetRows() + hing_costs_.GetRows());
}
Eigen::Index TrajOptQPProblem::getNumQPVars() const { return num_qp_vars_; }
Eigen::Index TrajOptQPProblem::getNumQPConstraints() const { return num_qp_cnts_; }

const std::vector<std::string>& TrajOptQPProblem::getNLPConstraintNames() const { return constraint_names_; }
const std::vector<std::string>& TrajOptQPProblem::getNLPCostNames() const { return cost_names_; }

const Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoxSize() { return box_size_; }
const Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getConstraintMeritCoeff() { return constraint_merit_coeff_; }

const Eigen::Ref<const Eigen::SparseMatrix<double>> TrajOptQPProblem::getHessian() { return hessian_; }
const Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getGradient() { return gradient_; }

const Eigen::Ref<const Eigen::SparseMatrix<double>> TrajOptQPProblem::getConstraintMatrix()
{
  return constraint_matrix_;
}
const Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsLower() { return bounds_lower_; }
const Eigen::Ref<const Eigen::VectorXd> TrajOptQPProblem::getBoundsUpper() { return bounds_upper_; }

}  // namespace trajopt_sqp
