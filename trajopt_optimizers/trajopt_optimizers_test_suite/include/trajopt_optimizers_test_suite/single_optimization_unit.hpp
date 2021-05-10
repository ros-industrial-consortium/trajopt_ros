/**
 * @file basic_optimization_unit.hpp
 * @brief Basic nonlinear optimization problem suite
 *
 * This code makes use of the tutorial problem from the IFOPT github page
 * https://github.com/ethz-adrl/ifopt
 * Credit to Alexander Winkler
 *
 * @author Randall Kliman
 * @date July 14th, 2020
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
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

TRAJOPT_IGNORE_WARNINGS_POP

const bool DEBUG = false;

namespace ifopt
{
using Eigen::Vector2d;

class ExVariables : public VariableSet
{
public:
  ExVariables() : ExVariables("var_set1"){};
  ExVariables(const std::string& name) : VariableSet(1, name) { x0_ = 1; }

  void SetVariables(const VectorXd& x) override { x0_ = x(0); };

  VectorXd GetValues() const override
  {
    VectorXd x(1);
    x << x0_;
    return x;
  };

  VecBound GetBounds() const override
  {
    VecBound bounds(static_cast<size_t>(GetRows()));
    bounds.at(0) = Bounds(-1.0, 1.0);
    return bounds;
  }

private:
  double x0_;
};

class ExCost : public CostTerm
{
public:
  ExCost() : ExCost("cost_term1") {}
  ExCost(const std::string& name) : CostTerm(name) {}

  double GetCost() const override
  {
    VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
    return x(0);
  };

  void FillJacobianBlock(std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "var_set1")
    {
      VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();

      jac.coeffRef(0, 0) = 1;  // derivative of cost w.r.t x0
    }
  }
};

}  // namespace ifopt

class SingleOptimization : public testing::TestWithParam<const char*>
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

    nlp_.AddVariableSet(std::make_shared<ifopt::ExVariables>());
    nlp_.AddCostSet(std::make_shared<ifopt::ExCost>());

    if (DEBUG)
    {
      nlp_.PrintCurrent();
      std::cout << "Jacobian: \n" << nlp_.GetJacobianOfConstraints() << std::endl;
    }
  }
};

/**
 * @brief Allows for any solver to run tests on the above class. The solver has to have a .Solve(ifopt::Problem&)
 * function
 * @param solver Any solver with a .Solve(ifopt::Problem&) function
 * @param nlp_opt An ifopt defined problem (like the one above)
 */
template <class T>
inline void runTests(T solver, ifopt::Problem nlp_opt)
{
  // Solve and test nlp
  solver.Solve(nlp_opt);
  Eigen::VectorXd x = nlp_opt.GetOptVariables()->GetValues();
  EXPECT_NEAR(x(0), -1.0, 1e-5);

  if (DEBUG)
  {
    std::cout << std::endl;
    std::cout << "x(0): " << x(0) << std::endl;
    std::cout << std::endl;
    nlp_opt.PrintCurrent();
  }
}
