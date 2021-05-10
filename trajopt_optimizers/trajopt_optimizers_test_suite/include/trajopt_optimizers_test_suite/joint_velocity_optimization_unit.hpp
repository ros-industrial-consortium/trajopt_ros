#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/costs/squared_cost.h>

const bool DEBUG = false;

class VelocityConstraintOptimization : public testing::TestWithParam<const char*>
{
public:
  // 1) Create the problem
  ifopt::Problem nlp_;

  void SetUp() override
  {
    if (DEBUG)
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    else
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);

    // 2) Add Variables
    std::vector<trajopt::JointPosition::ConstPtr> vars;
    std::vector<std::string> joint_names(7, "name");
    {
      auto pos = Eigen::VectorXd::Zero(7);
      auto var = std::make_shared<trajopt::JointPosition>(pos, joint_names, "Joint_Position_0");
      auto bounds = std::vector<ifopt::Bounds>(7, ifopt::NoBound);
      var->SetBounds(bounds);
      vars.push_back(var);
      nlp_.AddVariableSet(var);
    }
    for (int ind = 1; ind < 3; ind++)
    {
      auto pos = Eigen::VectorXd::Ones(7) * 10;
      auto var = std::make_shared<trajopt::JointPosition>(pos, joint_names, "Joint_Position_" + std::to_string(ind));
      auto bounds = std::vector<ifopt::Bounds>(7, ifopt::NoBound);
      var->SetBounds(bounds);
      vars.push_back(var);
      nlp_.AddVariableSet(var);
    }

    // 3) Add constraints
    Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(7);
    std::vector<trajopt::JointPosition::ConstPtr> start;
    start.push_back(vars.front());
    auto start_constraint = std::make_shared<trajopt::JointPosConstraint>(start_pos, start, "StartPosition");
    nlp_.AddConstraintSet(start_constraint);

    Eigen::VectorXd end_pos = Eigen::VectorXd::Ones(7) * 10;
    std::vector<trajopt::JointPosition::ConstPtr> end;
    end.push_back(vars.back());
    auto end_constraint = std::make_shared<trajopt::JointPosConstraint>(end_pos, end, "EndPosition");
    nlp_.AddConstraintSet(end_constraint);

    // 4) Add costs
    Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(7);
    auto vel_constraint = std::make_shared<trajopt::JointVelConstraint>(vel_target, vars, "jv");

    // Must link the variables to the constraint since that happens in AddConstraintSet
    vel_constraint->LinkWithVariables(nlp_.GetOptVariables());
    Eigen::VectorXd weights = Eigen::VectorXd::Ones(vel_constraint->GetRows()) * 0.1;
    auto vel_cost = std::make_shared<trajopt::SquaredCost>(vel_constraint, weights);
    nlp_.AddCostSet(vel_cost);

    if (DEBUG)
    {
      nlp_.PrintCurrent();
      std::cout << "Constraint Jacobian: \n" << nlp_.GetJacobianOfConstraints().toDense() << std::endl;
      std::cout << "Cost Jacobian: \n" << nlp_.GetJacobianOfCosts() << std::endl;
    }
  }
};

template <class T>
inline void runTests(T solver, ifopt::Problem nlp_opt)
{
  solver.Solve(nlp_opt);
  Eigen::VectorXd x = nlp_opt.GetOptVariables()->GetValues();

  for (Eigen::Index i = 0; i < 7; i++)
    EXPECT_NEAR(x[i], 0.0, 1e-5);
  for (Eigen::Index i = 7; i < 14; i++)
    EXPECT_NEAR(x[i], 5.0, 1e-1);
  for (Eigen::Index i = 14; i < 21; i++)
    EXPECT_NEAR(x[i], 10.0, 1e-5);
  if (DEBUG)
  {
    std::cout << x.transpose() << std::endl;
    nlp_opt.PrintCurrent();
  }
}
