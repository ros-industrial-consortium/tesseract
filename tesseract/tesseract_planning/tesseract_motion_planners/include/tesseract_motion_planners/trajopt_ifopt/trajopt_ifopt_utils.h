/**
 * @file trajopt_utils.h
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_UTILS_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ifopt/constraint_set.h>
#include <ifopt/problem.h>
#include <trajopt_ifopt/trajopt_ifopt.h>
#include <trajopt_ifopt/constraints/collision_evaluators.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/types.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>

namespace tesseract_planning
{
ifopt::ConstraintSet::Ptr createCartesianPositionConstraint(const CartesianWaypoint& cart_waypoint,
                                                            trajopt::JointPosition::ConstPtr var,
                                                            const trajopt::CartPosKinematicInfo::ConstPtr& kin_info,
                                                            const Eigen::VectorXd& coeffs);

bool addCartesianPositionConstraint(std::shared_ptr<ifopt::Problem> nlp,
                                    const CartesianWaypoint& cart_waypoint,
                                    trajopt::JointPosition::ConstPtr var,
                                    const trajopt::CartPosKinematicInfo::ConstPtr& kin_info,
                                    const Eigen::Ref<const Eigen::VectorXd>& coeff);

bool addCartesianPositionSquaredCost(std::shared_ptr<ifopt::Problem> nlp,
                                     const CartesianWaypoint& cart_waypoint,
                                     trajopt::JointPosition::ConstPtr var,
                                     const trajopt::CartPosKinematicInfo::ConstPtr& kin_info,
                                     const Eigen::Ref<const Eigen::VectorXd>& coeff);

ifopt::ConstraintSet::Ptr createJointPositionConstraint(const JointWaypoint& joint_waypoint,
                                                        trajopt::JointPosition::ConstPtr var,
                                                        const Eigen::VectorXd& /*coeffs*/);

bool addJointPositionConstraint(std::shared_ptr<ifopt::Problem> nlp,
                                const JointWaypoint& joint_waypoint,
                                trajopt::JointPosition::ConstPtr var,
                                const Eigen::Ref<const Eigen::VectorXd>& coeff);

bool addJointPositionSquaredCost(std::shared_ptr<ifopt::Problem> nlp,
                                 const JointWaypoint& joint_waypoint,
                                 trajopt::JointPosition::ConstPtr var,
                                 const Eigen::Ref<const Eigen::VectorXd>& coeff);

std::vector<ifopt::ConstraintSet::Ptr>
createCollisionConstraints(std::vector<trajopt::JointPosition::ConstPtr> vars,
                           const tesseract_environment::Environment::ConstPtr& env,
                           const ManipulatorInfo& manip_info,
                           const trajopt::TrajOptCollisionConfig::ConstPtr& config);

bool addCollisionConstraint(std::shared_ptr<ifopt::Problem> nlp,
                            std::vector<trajopt::JointPosition::ConstPtr> vars,
                            const tesseract_environment::Environment::ConstPtr& env,
                            const ManipulatorInfo& manip_info,
                            const trajopt::TrajOptCollisionConfig::ConstPtr& config);

bool addCollisionSquaredCost(std::shared_ptr<ifopt::Problem> nlp,
                             std::vector<trajopt::JointPosition::ConstPtr> vars,
                             const tesseract_environment::Environment::ConstPtr& env,
                             const ManipulatorInfo& manip_info,
                             const trajopt::TrajOptCollisionConfig::ConstPtr& config);

ifopt::ConstraintSet::Ptr createJointVelocityConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                                                        const std::vector<trajopt::JointPosition::ConstPtr>& vars,
                                                        const Eigen::VectorXd& coeffs);

bool addJointVelocityConstraint(std::shared_ptr<ifopt::Problem> nlp,
                                std::vector<trajopt::JointPosition::ConstPtr> vars,
                                const Eigen::Ref<const Eigen::VectorXd>& coeff);

bool addJointVelocitySquaredCost(std::shared_ptr<ifopt::Problem> nlp,
                                 std::vector<trajopt::JointPosition::ConstPtr> vars,
                                 const Eigen::Ref<const Eigen::VectorXd>& coeff);

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_UTILS_H
