/**
 * @file default_problem_generator.h
 * @brief Generates a trajopt problem from a planner request
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_PROBLEM_GENERATOR_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_PROBLEM_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ifopt/problem.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>

namespace tesseract_planning
{
std::shared_ptr<TrajOptIfoptProblem>
DefaultTrajoptProblemGenerator(const std::string& name,
                               const PlannerRequest& request,
                               const TrajOptIfoptPlanProfileMap& plan_profiles,
                               const TrajOptIfoptCompositeProfileMap& composite_profiles);

}  // namespace tesseract_planning
#endif
