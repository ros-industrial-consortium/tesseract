/**
 * @file TrajOptIfopt_default_plan_profile.h
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

#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>

namespace tesseract_planning
{
class TrajOptIfoptDefaultPlanProfile : public TrajOptIfoptPlanProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptDefaultPlanProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptDefaultPlanProfile>;

  TrajOptIfoptDefaultPlanProfile() = default;
  TrajOptIfoptDefaultPlanProfile(const tinyxml2::XMLElement& xml_element);

  Eigen::VectorXd cartesian_coeff{ Eigen::VectorXd::Constant(1, 1, 5) };
  Eigen::VectorXd joint_coeff{ Eigen::VectorXd::Constant(1, 1, 5) };
  TrajOptIfoptTermType term_type{ TrajOptIfoptTermType::CONSTRAINT };

  void apply(TrajOptIfoptProblem& problem,
             const CartesianWaypoint& cartesian_waypoint,
             const Instruction& parent_instruction,
             const ManipulatorInfo& manip_info,
             const std::vector<std::string>& active_links,
             int index) const override;

  void apply(TrajOptIfoptProblem& problem,
             const JointWaypoint& joint_waypoint,
             const Instruction& parent_instruction,
             const ManipulatorInfo& manip_info,
             const std::vector<std::string>& active_links,
             int index) const override;

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const override;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TrajOptIfopt_IFOPT_DEFAULT_PLAN_PROFILE_H
