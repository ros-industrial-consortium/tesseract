/**
 * @file trajopt_default_plan_profile.cpp
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

#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/instruction_type.h>
#include <trajopt_ifopt/trajopt_ifopt.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

namespace tesseract_planning
{
void TrajOptIfoptDefaultPlanProfile::apply(TrajOptIfoptProblem& problem,
                                           const CartesianWaypoint& cartesian_waypoint,
                                           const Instruction& parent_instruction,
                                           const ManipulatorInfo& manip_info,
                                           const std::vector<std::string>& active_links,
                                           int index) const
{
  assert(isPlanInstruction(parent_instruction));
  const auto* base_instruction = parent_instruction.cast_const<PlanInstruction>();
  assert(!(manip_info.empty() && base_instruction->getManipulatorInfo().empty()));
  const ManipulatorInfo& mi =
      (base_instruction->getManipulatorInfo().empty()) ? manip_info : base_instruction->getManipulatorInfo();

  const auto& env = problem.environment;
  auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), problem.manip_fwd_kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  // TODO: Fix 3rd arg, TCP, etc
  auto kin_info = std::make_shared<trajopt::CartPosKinematicInfo>(
      problem.manip_fwd_kin, adjacency_map, Eigen::Isometry3d::Identity(), manip_info.manipulator);

  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain)
   */
  auto it = std::find(active_links.begin(), active_links.end(), mi.working_frame);
  if (it != active_links.end())
  {
    CONSOLE_BRIDGE_logWarn("Dynamic cartesian terms are not supported by trajopt_ifopt. PRs welcome");
  }
  else
  {
    auto idx = static_cast<std::size_t>(index);
    switch (term_type)
    {
      case TrajOptIfoptTermType::CONSTRAINT:
        addCartesianPositionConstraint(problem.nlp, cartesian_waypoint, problem.vars[idx], kin_info, cartesian_coeff);

        break;
      case TrajOptIfoptTermType::SQUARED_COST:
        addCartesianPositionConstraint(problem.nlp, cartesian_waypoint, problem.vars[idx], kin_info, cartesian_coeff);
        break;
    }
  }
}

void TrajOptIfoptDefaultPlanProfile::apply(TrajOptIfoptProblem& problem,
                                           const JointWaypoint& joint_waypoint,
                                           const Instruction& /*parent_instruction*/,
                                           const ManipulatorInfo& /*manip_info*/,
                                           const std::vector<std::string>& /*active_links*/,
                                           int index) const
{
  auto idx = static_cast<std::size_t>(index);
  switch (term_type)
  {
    case TrajOptIfoptTermType::CONSTRAINT:
      addJointPositionConstraint(problem.nlp, joint_waypoint, problem.vars[idx], joint_coeff);
      break;
    case TrajOptIfoptTermType::SQUARED_COST:
      addJointPositionConstraint(problem.nlp, joint_waypoint, problem.vars[idx], joint_coeff);
      break;
  }
}

tinyxml2::XMLElement* TrajOptIfoptDefaultPlanProfile::toXML(tinyxml2::XMLDocument& /*doc*/) const
{
  throw std::runtime_error("TrajOptIfoptDefaultPlanProfile::toXML is not implemented!");
}

}  // namespace tesseract_planning
