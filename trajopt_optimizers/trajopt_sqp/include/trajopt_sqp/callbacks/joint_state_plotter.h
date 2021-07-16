/**
 * @file joint_state_plotter.h
 * @brief A callback to plot a set of JointPosition variables
 *
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
#ifndef TRAJOPT_SQP_INCLUDE_JOINT_STATE_PLOTTER_H_
#define TRAJOPT_SQP_INCLUDE_JOINT_STATE_PLOTTER_H_

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/problem.h>
#include <tesseract_visualization/visualization.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/sqp_callback.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/types.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

namespace trajopt_sqp
{
/**
 * @brief SQPCallback that plots joint positions as a Tesseract Trajectory
 */
class JointStatePlottingCallback : public trajopt_sqp::SQPCallback
{
public:
  using Ptr = std::shared_ptr<JointStatePlottingCallback>;
  using ConstPtr = std::shared_ptr<const JointStatePlottingCallback>;

  /**
   * @brief Constructor for the callback
   * @param plotter Plotter used to plot the joint state
   */
  JointStatePlottingCallback(tesseract_visualization::Visualization::Ptr plotter,
                             tesseract_environment::StateSolver::Ptr state_solver);

  /**
   * @brief Plot the joint_position variables as a tesseract trajectory
   * @param nlp Unused
   */
  void plot(const ifopt::Problem& nlp);

  /**
   * @brief Add a variable set to be plotted
   * @param joint_position JointPosition variable to be plotted. They should all be the same size
   */
  void addVariableSet(const trajopt_ifopt::JointPosition::ConstPtr& joint_position);

  /**
   * @brief Adds multiple variable sets to be plotted
   * @param joint_positions JointPosition variables to be plotted. They should all be the same size
   */
  void addVariableSet(const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& joint_positions);

  bool execute(const ifopt::Problem& nlp, const trajopt_sqp::SQPResults& /*sqp_results*/) override;

protected:
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> joint_positions_;
  tesseract_visualization::Visualization::Ptr plotter_;
  tesseract_environment::StateSolver::Ptr state_solver_;
};
}  // namespace trajopt_sqp

#endif
