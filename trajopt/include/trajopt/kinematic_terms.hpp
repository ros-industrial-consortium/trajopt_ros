#pragma once

#include <Eigen/Core>
#include <tesseract_core/basic_env.h>
#include <tesseract_core/basic_kin.h>
#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/sco_fwd.hpp>

namespace trajopt
{
using namespace sco;
typedef BasicArray<Var> VarArray;

/**
 * @brief Used to calculate the error for CartPoseTermInfo
 * This is converted to a cost or constraint using TrajOptCostFromErrFunc or TrajOptConstraintFromErrFunc
 */
struct DynamicCartPoseErrCalculator : public TrajOptVectorOfVector
{
  std::string target_;
  tesseract::BasicKinConstPtr manip_;
  tesseract::BasicEnvConstPtr env_;
  std::string link_;
  Eigen::Isometry3d tcp_;
  DynamicCartPoseErrCalculator(const std::string& target,
                        tesseract::BasicKinConstPtr manip,
                        tesseract::BasicEnvConstPtr env,
                        std::string link,
                        Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
    : target_(target), manip_(manip), env_(env), link_(link), tcp_(tcp)
  {
  }

  void Plot(const tesseract::BasicPlottingPtr& plotter, const VectorXd& dof_vals) override;

  VectorXd operator()(const VectorXd& dof_vals) const;
};

/**
 * @brief Used to calculate the error for StaticCartPoseTermInfo
 * This is converted to a cost or constraint using TrajOptCostFromErrFunc or TrajOptConstraintFromErrFunc
 */
struct CartPoseErrCalculator : public TrajOptVectorOfVector
{
  Eigen::Isometry3d pose_inv_;
  tesseract::BasicKinConstPtr manip_;
  tesseract::BasicEnvConstPtr env_;
  std::string link_;
  Eigen::Isometry3d tcp_;
  CartPoseErrCalculator(const Eigen::Isometry3d& pose,
                              tesseract::BasicKinConstPtr manip,
                              tesseract::BasicEnvConstPtr env,
                              std::string link,
                              Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
    : pose_inv_(pose.inverse()), manip_(manip), env_(env), link_(link), tcp_(tcp)
  {
  }

  void Plot(const tesseract::BasicPlottingPtr& plotter, const VectorXd& dof_vals) override;

  VectorXd operator()(const VectorXd& dof_vals) const;
};

/**
 * @brief Used to calculate the jacobian for CartVelTermInfo
 *
 */
struct CartVelJacCalculator : MatrixOfVector
{
  tesseract::BasicKinConstPtr manip_;
  tesseract::BasicEnvConstPtr env_;
  std::string link_;
  double limit_;
  Eigen::Isometry3d tcp_;
  CartVelJacCalculator(tesseract::BasicKinConstPtr manip,
                       tesseract::BasicEnvConstPtr env,
                       std::string link,
                       double limit,
                       Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
    : manip_(manip), env_(env), link_(link), limit_(limit), tcp_(tcp)
  {
  }

  MatrixXd operator()(const VectorXd& dof_vals) const;
};

/**
 * @brief  Used to calculate the error for CartVelTermInfo
 * This is converted to a cost or constraint using TrajOptCostFromErrFunc or TrajOptConstraintFromErrFunc
 */
struct CartVelErrCalculator : VectorOfVector
{
  tesseract::BasicKinConstPtr manip_;
  tesseract::BasicEnvConstPtr env_;
  std::string link_;
  double limit_;
  Eigen::Isometry3d tcp_;
  CartVelErrCalculator(tesseract::BasicKinConstPtr manip,
                    tesseract::BasicEnvConstPtr env,
                    std::string link,
                    double limit,
                    Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
    : manip_(manip), env_(env), link_(link), limit_(limit), tcp_(tcp)
  {
  }

  VectorXd operator()(const VectorXd& dof_vals) const;
};
}
