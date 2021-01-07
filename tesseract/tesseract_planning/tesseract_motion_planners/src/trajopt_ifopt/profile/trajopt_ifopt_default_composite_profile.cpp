/**
 * @file trajopt_default_composite_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>

namespace tesseract_planning
{
void TrajOptIfoptDefaultCompositeProfile::apply(TrajOptIfoptProblem& problem,
                                                int start_index,
                                                int end_index,
                                                const ManipulatorInfo& manip_info,
                                                const std::vector<std::string>& /*active_links*/,
                                                const std::vector<int>& /*fixed_indices*/) const
{
  const std::vector<trajopt::JointPosition::ConstPtr> vars(&problem.vars[static_cast<std::size_t>(start_index)],
                                                           &problem.vars[static_cast<std::size_t>(end_index)]);

  if (collision_constraint_config->type != tesseract_collision::CollisionEvaluatorType::NONE)
    addCollisionConstraint(problem.nlp, vars, problem.environment, manip_info, collision_constraint_config);

  if (collision_cost_config->type != tesseract_collision::CollisionEvaluatorType::NONE)
    addCollisionSquaredCost(problem.nlp, vars, problem.environment, manip_info, collision_cost_config);

  if (smooth_velocities)
    addJointVelocitySquaredCost(problem.nlp, vars, velocity_coeff);

  if (smooth_accelerations)
    CONSOLE_BRIDGE_logWarn("Acceleration smoothing not yet supported by trajopt_ifopt. PRs welcome");

  if (smooth_jerks)
    CONSOLE_BRIDGE_logWarn("Jerk smoothing not yet supported by trajopt_ifopt. PRs welcome");
}

tinyxml2::XMLElement* TrajOptIfoptDefaultCompositeProfile::toXML(tinyxml2::XMLDocument& /*doc*/) const
{
  throw std::runtime_error("TrajOptIfoptDefaultCompositeProfile::toXML is not implemented!");
}

}  // namespace tesseract_planning
