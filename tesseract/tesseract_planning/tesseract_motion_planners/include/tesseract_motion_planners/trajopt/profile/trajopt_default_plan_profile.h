/**
 * @file trajopt_default_plan_profile.h
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

#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt_sco/modeling_utils.hpp>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::TrajOptDefaultPlanProfile)
#endif  // SWIG

namespace tesseract_planning
{
class TrajOptDefaultPlanProfile : public TrajOptPlanProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptDefaultPlanProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptDefaultPlanProfile>;

  TrajOptDefaultPlanProfile() = default;
  ~TrajOptDefaultPlanProfile() override = default;
  TrajOptDefaultPlanProfile(const tinyxml2::XMLElement& xml_element);
  TrajOptDefaultPlanProfile(const TrajOptDefaultPlanProfile&) = default;
  TrajOptDefaultPlanProfile& operator=(const TrajOptDefaultPlanProfile&) = default;
  TrajOptDefaultPlanProfile(TrajOptDefaultPlanProfile&&) = default;
  TrajOptDefaultPlanProfile& operator=(TrajOptDefaultPlanProfile&&) = default;

  Eigen::VectorXd cartesian_coeff{ Eigen::VectorXd::Constant(1, 1, 5) };
  Eigen::VectorXd joint_coeff{ Eigen::VectorXd::Constant(1, 1, 5) };
  trajopt::TermType term_type{ trajopt::TermType::TT_CNT };

  /** @brief Error function that is set as a constraint for each timestep.
   *
   * This is a vector of std::tuple<Error Function, Error Function Jacobian, Constraint Type, Coeff>, the error
   * function, constraint type, and coeff is required, but the jacobian is optional (nullptr).
   *
   * Error Function:
   *   arg: VectorXd will be all of the joint values for one timestep.
   *   return: VectorXd of violations for each joint. Anything != 0 will be a violation
   *
   * Error Function Jacobian:
   *   arg: VectorXd will be all of the joint values for one timestep.
   *   return: Eigen::MatrixXd that represents the change in the error function with respect to joint values
   *
   * Error Constraint Type
   *
   * Coefficients/Weights
   *
   */
  std::vector<std::tuple<sco::VectorOfVector::func, sco::MatrixOfVector::func, sco::ConstraintType, Eigen::VectorXd>>
      constraint_error_functions;

  void apply(trajopt::ProblemConstructionInfo& pci,
             const CartesianWaypoint& cartesian_waypoint,
             const Instruction& parent_instruction,
             const ManipulatorInfo& manip_info,
             const std::vector<std::string>& active_links,
             int index) const override;

  void apply(trajopt::ProblemConstructionInfo& pci,
             const JointWaypoint& joint_waypoint,
             const Instruction& parent_instruction,
             const ManipulatorInfo& manip_info,
             const std::vector<std::string>& active_links,
             int index) const override;

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const override;

protected:
  void addConstraintErrorFunctions(trajopt::ProblemConstructionInfo& pci, int index) const;

  void addAvoidSingularity(trajopt::ProblemConstructionInfo& pci, const std::vector<int>& fixed_steps) const;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_PLAN_PROFILE_H
