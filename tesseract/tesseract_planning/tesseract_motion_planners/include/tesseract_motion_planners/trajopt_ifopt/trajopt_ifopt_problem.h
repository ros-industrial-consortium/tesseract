/**
 * @file trajopt_ifopt_problem.h
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PROBLEM_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PROBLEM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ifopt/problem.h>
#include <vector>
#include <memory>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>

namespace tesseract_planning
{
enum class TrajOptIfoptTermType
{
  CONSTRAINT,
  SQUARED_COST
};

struct TrajOptIfoptProblem
{
  // These are required for Tesseract to configure Descartes
  tesseract_environment::Environment::ConstPtr environment;
  tesseract_environment::EnvState::ConstPtr env_state;

  // Kinematic Objects
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_fwd_kin;
  tesseract_kinematics::InverseKinematics::ConstPtr manip_inv_kin;

  std::shared_ptr<ifopt::Problem> nlp;
  std::vector<trajopt::JointPosition::ConstPtr> vars;
};

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_PROBLEM_H
