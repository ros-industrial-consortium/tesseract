/**
 * @file trajopt_default_composite_profile.h
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

#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_COMPOSITE_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_COMPOSITE_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <trajopt_ifopt/constraints/collision_evaluators.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>

namespace tesseract_planning
{
class TrajOptIfoptDefaultCompositeProfile : public TrajOptIfoptCompositeProfile
{
public:
  TrajOptIfoptDefaultCompositeProfile() = default;
  TrajOptIfoptDefaultCompositeProfile(const tinyxml2::XMLElement& xml_element);

  /** @brief Configuration info for collisions that are modeled as costs */
  trajopt::TrajOptCollisionConfig::Ptr collision_cost_config;
  /** @brief Configuration info for collisions that are modeled as constraints */
  trajopt::TrajOptCollisionConfig::Ptr collision_constraint_config;
  /** @brief If true, a joint velocity cost with a target of 0 will be applied for all timesteps Default: true*/
  bool smooth_velocities = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd velocity_coeff;
  /** @brief If true, a joint acceleration cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_accelerations = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd acceleration_coeff;
  /** @brief If true, a joint jerk cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_jerks = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd jerk_coeff;

  /** @brief Set the resolution at which state validity needs to be verified in order for a motion between two states
   * to be considered valid in post checking of trajectory returned by TrajOptIfopt.
   *
   * The resolution is equal to longest_valid_segment_fraction * state_space.getMaximumExtent()
   *
   * Note: The planner takes the conservative of either longest_valid_segment_fraction or longest_valid_segment_length.
   */
  double longest_valid_segment_fraction = 0.01;  // 1%

  /** @brief Set the resolution at which state validity needs to be verified in order for a motion between two states
   * to be considered valid. If norm(state1 - state0) > longest_valid_segment_length.
   *
   * Note: This gets converted to longest_valid_segment_fraction.
   *       longest_valid_segment_fraction = longest_valid_segment_length / state_space.getMaximumExtent()
   */
  double longest_valid_segment_length = 0.5;

  /** @brief Special link collision cost distances */
  trajopt::SafetyMarginData::Ptr special_collision_cost{ nullptr };
  /** @brief Special link collision constraint distances */
  trajopt::SafetyMarginData::Ptr special_collision_constraint{ nullptr };

  void apply(TrajOptIfoptProblem& problem,
             int start_index,
             int end_index,
             const ManipulatorInfo& manip_info,
             const std::vector<std::string>& active_links,
             const std::vector<int>& fixed_indices) const override;

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const override;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_DEFAULT_COMPOSITE_PROFILE_H
