/**
 * @file trajopt_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
#include <vector>
#include <memory>
#include <ifopt/problem.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_problem.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/types.h>

namespace tesseract_planning
{
class TrajOptIfoptPlanProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptPlanProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptPlanProfile>;

  TrajOptIfoptPlanProfile() = default;
  virtual ~TrajOptIfoptPlanProfile() = default;
  TrajOptIfoptPlanProfile(const TrajOptIfoptPlanProfile&) = default;
  TrajOptIfoptPlanProfile& operator=(const TrajOptIfoptPlanProfile&) = default;
  TrajOptIfoptPlanProfile(TrajOptIfoptPlanProfile&&) = default;
  TrajOptIfoptPlanProfile& operator=(TrajOptIfoptPlanProfile&&) = default;

  virtual void apply(TrajOptIfoptProblem& problem,
                     const CartesianWaypoint& cartesian_waypoint,
                     const Instruction& parent_instruction,
                     const ManipulatorInfo& manip_info,
                     const std::vector<std::string>& active_links,
                     int index) const = 0;

  virtual void apply(TrajOptIfoptProblem& problem,
                     const JointWaypoint& joint_waypoint,
                     const Instruction& parent_instruction,
                     const ManipulatorInfo& manip_info,
                     const std::vector<std::string>& active_links,
                     int index) const = 0;

  virtual tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const = 0;
};

class TrajOptIfoptCompositeProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptIfoptCompositeProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptIfoptCompositeProfile>;

  TrajOptIfoptCompositeProfile() = default;
  virtual ~TrajOptIfoptCompositeProfile() = default;
  TrajOptIfoptCompositeProfile(const TrajOptIfoptCompositeProfile&) = default;
  TrajOptIfoptCompositeProfile& operator=(const TrajOptIfoptCompositeProfile&) = default;
  TrajOptIfoptCompositeProfile(TrajOptIfoptCompositeProfile&&) = default;
  TrajOptIfoptCompositeProfile& operator=(TrajOptIfoptCompositeProfile&&) = default;

  virtual void apply(TrajOptIfoptProblem& problem,
                     int start_index,
                     int end_index,
                     const ManipulatorInfo& manip_info,
                     const std::vector<std::string>& active_links,
                     const std::vector<int>& fixed_indices) const = 0;

  virtual tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const = 0;
};

using TrajOptIfoptCompositeProfileMap = std::unordered_map<std::string, TrajOptIfoptCompositeProfile::ConstPtr>;
using TrajOptIfoptPlanProfileMap = std::unordered_map<std::string, TrajOptIfoptPlanProfile::ConstPtr>;

}  // namespace tesseract_planning
#ifdef SWIG
%template(TrajOptIfoptCompositeProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract_planning::TrajOptIfoptCompositeProfile>>;
%template(TrajOptIfoptPlanProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract_planning::TrajOptIfoptPlanProfile>>;
#endif  // SWIG

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PROFILE_H
