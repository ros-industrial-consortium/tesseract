/**
 * @file types.h
 * @brief Kinematics types.
 *
 * @author Levi Armstrong
 * @date March 1, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_TYPES_H
#define TESSERACT_KINEMATICS_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>

namespace tesseract_kinematics
{
struct SynchronizableData
{
  std::string base_link_name;                   /**< Link name of first link in the kinematic chain */
  std::string tip_link_name;                    /**< Link name of last kink in the kinematic chain */
  std::vector<std::string> joint_names;         /**< List of joint names */
  std::vector<std::string> link_names;          /**< List of link names */
  std::vector<std::string> active_link_names;   /**< List of link names that move with changes in joint values */
  tesseract_common::KinematicLimits limits;     /**< Joint limits, velocity limits, and acceleration limits */
  std::vector<Eigen::Index> redundancy_indices; /**< Joint indicies that have redundancy (ex. revolute) */

  void clear()
  {
    base_link_name = "";
    tip_link_name = "";
    joint_names.clear();
    link_names.clear();
    active_link_names.clear();
    limits = tesseract_common::KinematicLimits();
    redundancy_indices.clear();
  }

  bool operator==(const SynchronizableData& rhs) const
  {
    bool equal = true;
    equal &= (base_link_name == rhs.base_link_name);
    equal &= (tip_link_name == rhs.tip_link_name);
    equal &= tesseract_common::isIdentical(joint_names, rhs.joint_names);
    equal &= tesseract_common::isIdentical(link_names, rhs.link_names);
    equal &= tesseract_common::isIdentical(active_link_names, rhs.active_link_names);
    equal &= (limits == rhs.limits);
    equal &= tesseract_common::isIdentical(redundancy_indices, rhs.redundancy_indices);
    return equal;
  }
  bool operator!=(const SynchronizableData& rhs) const { return !operator==(rhs); }
};

/** @brief The Universal Robot kinematic parameters */
struct URParameters
{
  URParameters() = default;
  URParameters(double d1, double a2, double a3, double d4, double d5, double d6)
    : d1(d1), a2(a2), a3(a3), d4(d4), d5(d5), d6(d6)
  {
  }

  double d1{ 0 };
  double a2{ 0 };
  double a3{ 0 };
  double d4{ 0 };
  double d5{ 0 };
  double d6{ 0 };
};

/** @brief The UR10 kinematic parameters */
const static URParameters UR10Parameters(0.1273, -0.612, -0.5723, 0.163941, 0.1157, 0.0922);

/** @brief The UR5 kinematic parameters */
const static URParameters UR5Parameters(0.089159, -0.42500, -0.39225, 0.10915, 0.09465, 0.0823);

/** @brief The UR3 kinematic parameters */
const static URParameters UR3Parameters(0.1519, -0.24365, -0.21325, 0.11235, 0.08535, 0.0819);

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_TYPES_H
