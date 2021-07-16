/**
 * @file sqp_callback.h
 * @brief Contains the base class for SQP callbacks
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
#ifndef TRAJOPT_SQP_INCLUDE_SQP_CALLBACK_H_
#define TRAJOPT_SQP_INCLUDE_SQP_CALLBACK_H_

#include <trajopt_sqp/types.h>
#include <ifopt/problem.h>

namespace trajopt_sqp
{
/**
 * @brief Base class for callbacks called during the SQP routine
 */
class SQPCallback
{
public:
  using Ptr = std::shared_ptr<SQPCallback>;
  using ConstPtr = std::shared_ptr<const SQPCallback>;

  /**
   * @brief This is the function called during the SQP
   * @param nlp The ifopt::Problem being optimized
   * @param sqp_results The latest SQPResults
   * @return Returning false will stop the optimization
   */
  virtual bool execute(const ifopt::Problem& nlp, const SQPResults& sqp_results) = 0;
};
}  // namespace trajopt_sqp

#endif
