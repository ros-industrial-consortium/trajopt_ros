/**
 * @file joint_position_optimization_unit.cpp
 * @brief A joint position constraint unit test
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
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

const bool DEBUG = false;

class JointPositionOptimization : public testing::TestWithParam<const char*>
{
public:
  void SetUp() override
  {
    if (DEBUG)
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    else
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);
  }
};

void runJointPositionOptimizationTest(trajopt_sqp::QPProblem::Ptr qp_problem)
{
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(DEBUG);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
  qp_solver->solver_.settings()->setMaxIteration(8192);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  // 2) Add Variables
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
  for (int ind = 0; ind < 2; ind++)
  {
    auto pos = Eigen::VectorXd::Zero(7);
    std::vector<std::string> joint_names(7, "name");
    auto var =
        std::make_shared<trajopt_ifopt::JointPosition>(pos, joint_names, "Joint_Position_" + std::to_string(ind));
    vars.push_back(var);
    qp_problem->addVariableSet(var);
  }

  // 3) Add constraints
  Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(7);
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> start;
  start.push_back(vars.front());

  auto start_constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(start_pos, start, "StartPosition");
  qp_problem->addConstraintSet(start_constraint);

  Eigen::VectorXd end_pos = Eigen::VectorXd::Ones(7);
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> end;
  end.push_back(vars.back());
  auto end_constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(end_pos, end, "EndPosition");
  qp_problem->addConstraintSet(end_constraint);

  qp_problem->setup();

  // solve
  solver.verbose = DEBUG;
  solver.solve(qp_problem);
  Eigen::VectorXd x = qp_problem->getVariableValues();

  for (Eigen::Index i = 0; i < 7; i++)
    EXPECT_NEAR(x[i], 0.0, 1e-5);
  for (Eigen::Index i = 7; i < 14; i++)
    EXPECT_NEAR(x[i], 1.0, 1e-5);

  if (DEBUG)
    qp_problem->print();
}

/**
 * @brief Applies a joint position constraint and solves the problem with IPOPT
 */
TEST_F(JointPositionOptimization, joint_position_optimization_ipopt)  // NOLINT
{
  // 1) create Problem
  ifopt::Problem nlp_ipopt;

  // 2) Add Variables
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
  for (int ind = 0; ind < 2; ind++)
  {
    auto pos = Eigen::VectorXd::Zero(7);
    std::vector<std::string> joint_names(7, "name");
    auto var =
        std::make_shared<trajopt_ifopt::JointPosition>(pos, joint_names, "Joint_Position_" + std::to_string(ind));
    vars.push_back(var);
    nlp_ipopt.AddVariableSet(var);
  }

  // 3) Add constraints
  Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(7);
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> start;
  start.push_back(vars.front());

  auto start_constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(start_pos, start, "StartPosition");
  nlp_ipopt.AddConstraintSet(start_constraint);

  Eigen::VectorXd end_pos = Eigen::VectorXd::Ones(7);
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> end;
  end.push_back(vars.back());
  auto end_constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(end_pos, end, "EndPosition");
  nlp_ipopt.AddConstraintSet(end_constraint);

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
    EXPECT_NEAR(x[i], 1.0, 1e-5);
  if (DEBUG)
  {
    std::cout << x.transpose() << std::endl;
    nlp_ipopt.PrintCurrent();
  }
}

/**
 * @brief Applies a joint position constraint and solves the ifopt problem with trajopt_sqp
 */
TEST_F(JointPositionOptimization, joint_position_optimization_ifopt_problem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointPositionOptimization, joint_position_optimization_ifopt_problem");
  auto qp_problem = std::make_shared<trajopt_sqp::IfoptQPProblem>();
  runJointPositionOptimizationTest(qp_problem);
}

/**
 * @brief Applies a joint position constraint and solves the ifopt problem with trajopt_sqp
 */
TEST_F(JointPositionOptimization, joint_position_optimization_trajopt_problem)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("JointPositionOptimization, joint_position_optimization_trajopt_problem");
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
  runJointPositionOptimizationTest(qp_problem);
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
