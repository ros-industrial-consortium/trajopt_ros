/**
 * @file joint_velocity_optimization_unit.cpp
 * @brief A joint velocity constraint unit test
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>

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
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
    std::vector<std::string> joint_names(7, "name");
    {
      auto pos = Eigen::VectorXd::Zero(7);
      auto var = std::make_shared<trajopt_ifopt::JointPosition>(pos, joint_names, "Joint_Position_0");
      auto bounds = std::vector<ifopt::Bounds>(7, ifopt::NoBound);
      var->SetBounds(bounds);
      vars.push_back(var);
      nlp_.AddVariableSet(var);
    }
    for (int ind = 1; ind < 3; ind++)
    {
      auto pos = Eigen::VectorXd::Ones(7) * 10;
      auto var =
          std::make_shared<trajopt_ifopt::JointPosition>(pos, joint_names, "Joint_Position_" + std::to_string(ind));
      auto bounds = std::vector<ifopt::Bounds>(7, ifopt::NoBound);
      var->SetBounds(bounds);
      vars.push_back(var);
      nlp_.AddVariableSet(var);
    }

    // 3) Add constraints
    Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(7);
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> start;
    start.push_back(vars.front());
    auto start_constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(start_pos, start, "StartPosition");
    nlp_.AddConstraintSet(start_constraint);

    Eigen::VectorXd end_pos = Eigen::VectorXd::Ones(7) * 10;
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> end;
    end.push_back(vars.back());
    auto end_constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(end_pos, end, "EndPosition");
    nlp_.AddConstraintSet(end_constraint);

    // 4) Add costs
    Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(7);
    auto vel_constraint = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, vars, "jv");

    // Must link the variables to the constraint since that happens in AddConstraintSet
    vel_constraint->LinkWithVariables(nlp_.GetOptVariables());
    Eigen::VectorXd weights = Eigen::VectorXd::Constant(vel_constraint->GetRows(), 0.01);
    auto vel_cost = std::make_shared<trajopt_ifopt::SquaredCost>(vel_constraint, weights);
    nlp_.AddCostSet(vel_cost);

    if (DEBUG)
    {
      nlp_.PrintCurrent();
      std::cout << "Constraint Jacobian: \n" << nlp_.GetJacobianOfConstraints().toDense() << std::endl;
      std::cout << "Cost Jacobian: \n" << nlp_.GetJacobianOfCosts() << std::endl;
    }
  }
};

/** @brief Joint position constraints with a squared velocity cost in between. Optimized using ipopt */
TEST_F(VelocityConstraintOptimization, velocity_constraint_optimization_ipopt)  // NOLINT
{
  ifopt::Problem nlp_ipopt(nlp_);
  ifopt::IpoptSolver solver;
  solver.SetOption("derivative_test", "first-order");
  solver.SetOption("linear_solver", "mumps");
  //  ipopt.SetOption("jacobian_approximation", "finite-difference-values");
  solver.SetOption("jacobian_approximation", "exact");
  solver.SetOption("print_level", 5);

  // solve
  solver.Solve(nlp_ipopt);
  Eigen::VectorXd x = nlp_ipopt.GetOptVariables()->GetValues();

  for (Eigen::Index i = 0; i < 7; i++)
    EXPECT_NEAR(x[i], 0.0, 1e-5);
  for (Eigen::Index i = 7; i < 14; i++)
    EXPECT_NEAR(x[i], 5.0, 1e-1);
  for (Eigen::Index i = 14; i < 21; i++)
    EXPECT_NEAR(x[i], 10.0, 1e-5);
  if (DEBUG)
  {
    std::cout << x.transpose() << std::endl;
    nlp_ipopt.PrintCurrent();
  }
}

/** @brief Joint position constraints with a squared velocity cost in between. Optimized using trajopt_sqp */
TEST_F(VelocityConstraintOptimization, velocity_constraint_optimization_ifopt_problem)  // NOLINT
{
  ifopt::Problem nlp_trajopt_sqp(nlp_);
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(DEBUG);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);
  qp_solver->solver_.settings()->setMaxIteration(8192);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);

  // Create problem
  auto qp_problem = std::make_shared<trajopt_sqp::IfoptQPProblem>(nlp_);

  // 6) solve
  solver.verbose = DEBUG;
  solver.solve(qp_problem);
  Eigen::VectorXd x = qp_problem->getVariableValues();

  for (Eigen::Index i = 0; i < 7; i++)
    EXPECT_NEAR(x[i], 0.0, 1e-3);
  for (Eigen::Index i = 7; i < 14; i++)
    EXPECT_NEAR(x[i], 5.0, 1e-1);
  for (Eigen::Index i = 14; i < 21; i++)
    EXPECT_NEAR(x[i], 10.0, 1e-3);
  if (DEBUG)
    qp_problem->print();
}

TEST_F(VelocityConstraintOptimization, velocity_constraint_optimization_trajopt_problem)  // NOLINT
{
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(DEBUG);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);
  qp_solver->solver_.settings()->setMaxIteration(8192);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);

  // Create problem
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();

  // 2) Add Variables
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
  std::vector<std::string> joint_names(7, "name");
  {
    auto pos = Eigen::VectorXd::Zero(7);
    auto var = std::make_shared<trajopt_ifopt::JointPosition>(pos, joint_names, "Joint_Position_0");
    auto bounds = std::vector<ifopt::Bounds>(7, ifopt::NoBound);
    var->SetBounds(bounds);
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }
  for (int ind = 1; ind < 3; ind++)
  {
    auto pos = Eigen::VectorXd::Ones(7) * 10;
    auto var =
        std::make_shared<trajopt_ifopt::JointPosition>(pos, joint_names, "Joint_Position_" + std::to_string(ind));
    auto bounds = std::vector<ifopt::Bounds>(7, ifopt::NoBound);
    var->SetBounds(bounds);
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }

  // 3) Add constraints
  Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(7);
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> start;
  start.push_back(vars.front());
  auto start_constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(start_pos, start, "StartPosition");
  nlp_.AddConstraintSet(start_constraint);

  Eigen::VectorXd end_pos = Eigen::VectorXd::Ones(7) * 10;
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> end;
  end.push_back(vars.back());
  auto end_constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(end_pos, end, "EndPosition");
  qp_problem->addConstraintSet(end_constraint);

  // 4) Add costs
  Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(7);
  auto vel_constraint = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, vars, "jv");

  // Must link the variables to the constraint since that happens in AddConstraintSet
  //  vel_constraint->LinkWithVariables(nlp_.GetOptVariables());
  //  Eigen::VectorXd weights = Eigen::VectorXd::Constant(vel_constraint->GetRows(), 0.01);
  //  auto vel_cost = std::make_shared<trajopt_ifopt::SquaredCost>(vel_constraint, weights);
  qp_problem->addCostSet(vel_constraint, trajopt_sqp::CostPenaltyType::SQUARED);

  qp_problem->setup();

  // 6) solve
  solver.verbose = DEBUG;
  solver.solve(qp_problem);
  Eigen::VectorXd x = qp_problem->getVariableValues();

  for (Eigen::Index i = 0; i < 7; i++)
    EXPECT_NEAR(x[i], 0.0, 1e-3);
  for (Eigen::Index i = 7; i < 14; i++)
    EXPECT_NEAR(x[i], 5.0, 1e-1);
  for (Eigen::Index i = 14; i < 21; i++)
    EXPECT_NEAR(x[i], 10.0, 1e-3);

  if (DEBUG)
    qp_problem->print();
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
