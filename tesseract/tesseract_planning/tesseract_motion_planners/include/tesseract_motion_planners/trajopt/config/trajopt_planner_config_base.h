/**
 * @file trajopt_planner_config_base.h
 * @brief The base class for TrajOpt planner configuration
 *
 * @author Michael Ripperger
 * @date September 16, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_PLANNER_CONFIG_BASE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_PLANNER_CONFIG_BASE_H

#include <trajopt/problem_description.hpp>
#include <tesseract_motion_planners/core/waypoint.h>

namespace tesseract_motion_planners
{
/**
 * @brief The TrajOptPlannerConfigBase struct
 */
struct TrajOptPlannerConfigBase
{
  using Ptr = std::shared_ptr<TrajOptPlannerConfigBase>;

  explicit TrajOptPlannerConfigBase() = default;
  virtual ~TrajOptPlannerConfigBase() {}

  /**
   * @brief Generates the TrajOpt problem and saves the result internally
   * @return True on success, false on failure
   */
  virtual bool generate() = 0;

  /** @brief Optimization parameters to be used (Optional) */
  sco::BasicTrustRegionSQPParameters params;

  /** @brief Callback functions called on each iteration of the optimization (Optional) */
  std::vector<sco::Optimizer::Callback> callbacks;

  /** @brief Trajopt problem to be solved (Required) */
  trajopt::TrajOptProb::Ptr prob;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_PLANNER_CONFIG_BASE_H
