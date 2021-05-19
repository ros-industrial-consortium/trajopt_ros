#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <constants.h>
#include <cmath>
#include <Eigen/SparseCore>
#include <fstream>
#include <csignal>
#include <iomanip>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/osqp_interface.hpp>
#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

namespace sco
{
const double OSQP_INFINITY = std::numeric_limits<double>::infinity();
const bool SUPER_DEBUG_MODE = false;

Model::Ptr createOSQPModel()
{
  auto out = std::make_shared<OSQPModel>();
  return out;
}

OSQPModel::OSQPModel() : P_(nullptr), A_(nullptr)
{
  // Define Solver settings as default
  // see https://osqp.org/docs/interfaces/solver_settings.html#solver-settings
  osqp_set_default_settings(&osqp_settings_);
  // tuning parameters to be less accurate, but add a polishing step
  osqp_settings_.eps_abs = 1e-4;
  osqp_settings_.eps_rel = 1e-6;
  osqp_settings_.max_iter = 8192;
  osqp_settings_.polish = 1;
  osqp_settings_.verbose = SUPER_DEBUG_MODE;
  osqp_settings_.adaptive_rho = false;
}
OSQPModel::~OSQPModel()
{
  // The osqp_workspace_ is managed by osqp but its members are not so must clean up.
  if (osqp_workspace_ != nullptr)
    osqp_cleanup(osqp_workspace_);

  // Clean up memory
  for (Var& var : vars_)
    var.var_rep->removed = true;
  for (Cnt& cnt : cnts_)
    cnt.cnt_rep->removed = true;
  update();
}

Var OSQPModel::addVar(const std::string& name)
{
  vars_.push_back(std::make_shared<VarRep>(vars_.size(), name, this));
  lbs_.push_back(-OSQP_INFINITY);
  ubs_.push_back(OSQP_INFINITY);
  return vars_.back();
}

Cnt OSQPModel::addEqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  cnts_.push_back(std::make_shared<CntRep>(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(EQ);
  return cnts_.back();
}

Cnt OSQPModel::addIneqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  cnts_.push_back(std::make_shared<CntRep>(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(INEQ);
  return cnts_.back();
}

Cnt OSQPModel::addIneqCnt(const QuadExpr&, const std::string& /*name*/) { throw std::runtime_error("NOT IMPLEMENTED"); }

void OSQPModel::removeVars(const VarVector& vars)
{
  SizeTVec inds;
  vars2inds(vars, inds);
  for (auto& var : vars)
    var.var_rep->removed = true;
}

void OSQPModel::removeCnts(const CntVector& cnts)
{
  SizeTVec inds;
  cnts2inds(cnts, inds);
  for (auto& cnt : cnts)
    cnt.cnt_rep->removed = true;
}

void OSQPModel::updateObjective()
{
  const size_t n = vars_.size();
  osqp_data_.n = static_cast<c_int>(n);

  Eigen::SparseMatrix<double> sm;
  exprToEigen(objective_, sm, q_, static_cast<int>(n), true);

  // Copy triangular upper into empty matrix
  Eigen::SparseMatrix<double> triangular_sm;
  triangular_sm = sm.triangularView<Eigen::Upper>();
  if (SUPER_DEBUG_MODE)
    std::cout << std::fixed << std::setprecision(3) << "OSQP Hessian:\n" << triangular_sm.toDense() << std::endl;
  eigenToCSC(triangular_sm, P_row_indices_, P_column_pointers_, P_csc_data_);

  P_.reset(csc_matrix(osqp_data_.n,
                      osqp_data_.n,
                      static_cast<c_int>(P_csc_data_.size()),
                      P_csc_data_.data(),
                      P_row_indices_.data(),
                      P_column_pointers_.data()));

  osqp_data_.P = P_.get();
  osqp_data_.q = q_.data();

  if (SUPER_DEBUG_MODE)
  {
    Eigen::Map<Eigen::VectorXd> q_vec(q_.data(), q_.size());
    std::cout << std::fixed << std::setprecision(3) << "OSQP Gradient: " << q_vec.transpose() << std::endl;
  }
}

void OSQPModel::updateConstraints()
{
  const size_t n = vars_.size();
  const size_t m = cnts_.size();
  const auto n_int = static_cast<int>(n);
  const auto m_int = static_cast<int>(m);

  osqp_data_.m = static_cast<c_int>(m) + static_cast<c_int>(n);

  Eigen::SparseMatrix<double> sm;
  Eigen::VectorXd v;
  exprToEigen(cnt_exprs_, sm, v, static_cast<int>(n));
  Eigen::SparseMatrix<double> sm_e(n_int + m_int, n_int);
  Eigen::SparseMatrix<double> sm_e2 = sm;
  sm.conservativeResize(m_int + n_int, Eigen::NoChange_t(n));

  l_.clear();
  l_.resize(m + n, -OSQP_INFINITY);
  u_.clear();
  u_.resize(m + n, OSQP_INFINITY);

  for (std::size_t i_cnt = 0; i_cnt < m; ++i_cnt)
  {
    l_[i_cnt] = (cnt_types_[i_cnt] == INEQ) ? -OSQP_INFINITY : v[static_cast<Eigen::Index>(i_cnt)];
    u_[i_cnt] = v[static_cast<Eigen::Index>(i_cnt)];
  }

  for (std::size_t i_bnd = 0; i_bnd < n; ++i_bnd)
  {
    l_[i_bnd + m] = fmax(lbs_[i_bnd], -OSQP_INFINITY);
    u_[i_bnd + m] = fmin(ubs_[i_bnd], OSQP_INFINITY);
    sm.insert(static_cast<Eigen::Index>(i_bnd + m), static_cast<Eigen::Index>(i_bnd)) = 1.;
  }
  if (SUPER_DEBUG_MODE)
    std::cout << std::fixed << std::setprecision(3) << "OSQP Constraint Matrix:\n" << sm.toDense() << std::endl;
  eigenToCSC(sm, A_row_indices_, A_column_pointers_, A_csc_data_);

  A_.reset(csc_matrix(osqp_data_.m,
                      osqp_data_.n,
                      static_cast<c_int>(A_csc_data_.size()),
                      A_csc_data_.data(),
                      A_row_indices_.data(),
                      A_column_pointers_.data()));

  osqp_data_.A = A_.get();

  if (SUPER_DEBUG_MODE)
  {
    Eigen::Map<Eigen::VectorXd> l_vec(l_.data(), static_cast<Eigen::Index>(l_.size()));
    Eigen::Map<Eigen::VectorXd> u_vec(u_.data(), static_cast<Eigen::Index>(u_.size()));
    std::cout << "OSQP Lower Bounds: " << l_vec.transpose() << std::endl;
    std::cout << "OSQP Upper Bounds: " << u_vec.transpose() << std::endl;
  }
  osqp_data_.l = l_.data();
  osqp_data_.u = u_.data();
}

void OSQPModel::createOrUpdateSolver()
{
  updateObjective();
  updateConstraints();

  // TODO atm we are not updating the workspace, but recreating it each time.
  // In the future, we will checking sparsity did not change and update instead
  if (osqp_workspace_ != nullptr)
    osqp_cleanup(osqp_workspace_);

  // Setup workspace - this should be called only once
  auto ret = osqp_setup(&osqp_workspace_, &osqp_data_, &osqp_settings_);
  if (ret)
  {
    // In this case, no data got allocated, so don't try to free it.
    if (ret == OSQP_DATA_VALIDATION_ERROR || ret == OSQP_SETTINGS_VALIDATION_ERROR)
      osqp_workspace_ = nullptr;
    throw std::runtime_error("Could not initialize OSQP: error " + std::to_string(ret));
  }
}

void OSQPModel::update()
{
  {
    std::size_t inew = 0;
    for (std::size_t iold = 0; iold < vars_.size(); ++iold)
    {
      Var& var = vars_[iold];
      if (!var.var_rep->removed)
      {
        vars_[inew] = var;
        lbs_[inew] = lbs_[iold];
        ubs_[inew] = ubs_[iold];
        var.var_rep->index = inew;
        ++inew;
      }
      else
      {
        var.var_rep = nullptr;
      }
    }
    vars_.resize(inew);
    lbs_.resize(inew);
    ubs_.resize(inew);
  }
  {
    std::size_t inew = 0;
    for (std::size_t iold = 0; iold < cnts_.size(); ++iold)
    {
      Cnt& cnt = cnts_[iold];
      if (!cnt.cnt_rep->removed)
      {
        cnts_[inew] = cnt;
        cnt_exprs_[inew] = cnt_exprs_[iold];
        cnt_types_[inew] = cnt_types_[iold];
        cnt.cnt_rep->index = inew;
        ++inew;
      }
      else
      {
        cnt.cnt_rep = nullptr;
      }
    }
    cnts_.resize(inew);
    cnt_exprs_.resize(inew);
    cnt_types_.resize(inew);
  }
}

void OSQPModel::setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper)
{
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    const std::size_t varind = vars[i].var_rep->index;
    lbs_[varind] = lower[i];
    ubs_[varind] = upper[i];
  }
}
DblVec OSQPModel::getVarValues(const VarVector& vars) const
{
  DblVec out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    const std::size_t varind = vars[i].var_rep->index;
    out[i] = solution_[varind];
  }
  return out;
}

CvxOptStatus OSQPModel::optimize()
{
  update();
  try
  {
    createOrUpdateSolver();
  }
  catch (std::exception& e)
  {
    std::cout << "OSQP Setup failed with error: " << e.what() << std::endl;
    return CVX_FAILED;
  }

  // Solve Problem
  const c_int retcode = osqp_solve(osqp_workspace_);

  if (retcode == 0)
  {
    // opt += m_objective.affexpr.constant;
    solution_ = DblVec(osqp_workspace_->solution->x, osqp_workspace_->solution->x + vars_.size());

    if (SUPER_DEBUG_MODE)
    {
      Eigen::Map<Eigen::VectorXd> solution_vec(solution_.data(), static_cast<Eigen::Index>(solution_.size()));
      std::cout << "Solution: " << solution_vec.transpose() << std::endl;
    }

    auto status = static_cast<int>(osqp_workspace_->info->status_val);
    if (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE)
      return CVX_SOLVED;
    if (status == OSQP_PRIMAL_INFEASIBLE || status == OSQP_PRIMAL_INFEASIBLE_INACCURATE ||
        status == OSQP_DUAL_INFEASIBLE || status == OSQP_DUAL_INFEASIBLE_INACCURATE)
      return CVX_INFEASIBLE;
  }
  return CVX_FAILED;
}
void OSQPModel::setObjective(const AffExpr& expr) { objective_.affexpr = expr; }
void OSQPModel::setObjective(const QuadExpr& expr) { objective_ = expr; }

VarVector OSQPModel::getVars() const { return vars_; }

void OSQPModel::writeToFile(const std::string& fname) const
{
  std::ofstream outStream(fname);
  outStream << "\\ Generated by trajopt_sco with backend OSQP\n";
  outStream << "Minimize\n";
  outStream << objective_;
  outStream << "Subject To\n";
  for (std::size_t i = 0; i < cnt_exprs_.size(); ++i)
  {
    std::string op = (cnt_types_[i] == INEQ) ? " <= " : " = ";
    outStream << cnt_exprs_[i] << op << 0 << "\n";
  }

  outStream << "Bounds\n";
  for (std::size_t i = 0; i < vars_.size(); ++i)
  {
    outStream << lbs_[i] << " <= " << vars_[i] << " <= " << ubs_[i] << "\n";
  }
  outStream << "End";
}
}  // namespace sco
