/**
 * @file trajopt_ifopt_utils.cpp
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
#include <trajopt_ifopt/trajopt_ifopt.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_problem.h>
#include <tesseract_command_language/types.h>
#include <trajopt_ifopt/constraints/collision_evaluators.h>

#include <ifopt/problem.h>

namespace tesseract_planning
{
ifopt::ConstraintSet::Ptr createCartesianPositionConstraint(const CartesianWaypoint& cart_waypoint,
                                                            trajopt::JointPosition::ConstPtr var,
                                                            const trajopt::CartPosKinematicInfo::ConstPtr& kin_info,
                                                            const Eigen::VectorXd& /*coeffs*/)
{
  auto constraint =
      std::make_shared<trajopt::CartPosConstraint>(cart_waypoint.waypoint, kin_info, var, "CartPos_" + var->GetName());
  return constraint;
}

bool addCartesianPositionConstraint(std::shared_ptr<ifopt::Problem> nlp,
                                    const CartesianWaypoint& cart_waypoint,
                                    trajopt::JointPosition::ConstPtr var,
                                    const trajopt::CartPosKinematicInfo::ConstPtr& kin_info,
                                    const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  auto constraint = createCartesianPositionConstraint(cart_waypoint, var, kin_info, coeff);
  nlp->AddConstraintSet(constraint);
  return true;
}

bool addCartesianPositionSquaredCost(std::shared_ptr<ifopt::Problem> nlp,
                                     const CartesianWaypoint& cart_waypoint,
                                     trajopt::JointPosition::ConstPtr var,
                                     const trajopt::CartPosKinematicInfo::ConstPtr& kin_info,
                                     const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  auto constraint = createCartesianPositionConstraint(cart_waypoint, var, kin_info, coeff);

  // Must link the variables to the constraint since that happens in AddConstraintSet
  constraint->LinkWithVariables(nlp->GetOptVariables());
  auto cost = std::make_shared<trajopt::SquaredCost>(constraint, coeff);
  nlp->AddCostSet(cost);
  return true;
}

ifopt::ConstraintSet::Ptr createJointPositionConstraint(const JointWaypoint& joint_waypoint,
                                                        trajopt::JointPosition::ConstPtr var,
                                                        const Eigen::VectorXd& /*coeffs*/)
{
  assert(var);
  std::vector<trajopt::JointPosition::ConstPtr> vars(1, var);

  ifopt::ConstraintSet::Ptr constraint;
  if (!joint_waypoint.isToleranced())
  {
    constraint =
        std::make_shared<trajopt::JointPosConstraint>(joint_waypoint.waypoint, vars, "JointPos_" + var->GetName());
  }
  else
  {
    Eigen::VectorXd lower_limit = joint_waypoint.waypoint + joint_waypoint.lower_tolerance;
    Eigen::VectorXd upper_limit = joint_waypoint.waypoint + joint_waypoint.upper_tolerance;
    auto bounds = trajopt::toBounds(lower_limit, upper_limit);
    constraint = std::make_shared<trajopt::JointPosConstraint>(bounds, vars, "JointPos_" + var->GetName());
  }

  return constraint;
}

bool addJointPositionConstraint(std::shared_ptr<ifopt::Problem> nlp,
                                const JointWaypoint& joint_waypoint,
                                trajopt::JointPosition::ConstPtr var,
                                const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  auto constraint = createJointPositionConstraint(joint_waypoint, var, coeff);
  nlp->AddConstraintSet(constraint);
  return true;
}

bool addJointPositionSquaredCost(std::shared_ptr<ifopt::Problem> nlp,
                                 const JointWaypoint& joint_waypoint,
                                 trajopt::JointPosition::ConstPtr var,
                                 const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  auto vel_constraint = createJointPositionConstraint(joint_waypoint, var, coeff);

  // Must link the variables to the constraint since that happens in AddConstraintSet
  vel_constraint->LinkWithVariables(nlp->GetOptVariables());
  auto vel_cost = std::make_shared<trajopt::SquaredCost>(vel_constraint, coeff);
  nlp->AddCostSet(vel_cost);
  return true;
}

std::vector<ifopt::ConstraintSet::Ptr>
createCollisionConstraints(std::vector<trajopt::JointPosition::ConstPtr> vars,
                           const tesseract_environment::Environment::ConstPtr& env,
                           const ManipulatorInfo& manip_info,
                           const trajopt::TrajOptCollisionConfig::ConstPtr& config)
{
  std::vector<ifopt::ConstraintSet::Ptr> constraints;
  if (config->type == tesseract_collision::CollisionEvaluatorType::NONE)
    return constraints;

  if (config->type != tesseract_collision::CollisionEvaluatorType::DISCRETE)
    CONSOLE_BRIDGE_logWarn("Only Single timestep collision is supported for trajopt_ifopt. PRs welcome");

  // Add a collision cost for all steps
  for (const auto& var : vars)
  {
    auto kin = env->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);
    auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
        env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

    auto collision_evaluator = std::make_shared<trajopt::DiscreteCollisionEvaluator>(
        kin, env, adjacency_map, Eigen::Isometry3d::Identity(), *config);

    constraints.push_back(std::make_shared<trajopt::CollisionConstraintIfopt>(collision_evaluator, var));
  }

  return constraints;
}

bool addCollisionConstraint(std::shared_ptr<ifopt::Problem> nlp,
                            std::vector<trajopt::JointPosition::ConstPtr> vars,
                            const tesseract_environment::Environment::ConstPtr& env,
                            const ManipulatorInfo& manip_info,
                            const trajopt::TrajOptCollisionConfig::ConstPtr& config)
{
  auto constraints = createCollisionConstraints(vars, env, manip_info, config);
  for (auto& constraint : constraints)
    nlp->AddConstraintSet(constraint);
  return true;
}

bool addCollisionSquaredCost(std::shared_ptr<ifopt::Problem> nlp,
                             std::vector<trajopt::JointPosition::ConstPtr> vars,
                             const tesseract_environment::Environment::ConstPtr& env,
                             const ManipulatorInfo& manip_info,
                             const trajopt::TrajOptCollisionConfig::ConstPtr& config)
{
  auto constraints = createCollisionConstraints(vars, env, manip_info, config);
  for (auto& constraint : constraints)
  {
    Eigen::VectorXd coeff;  // todo

    // Must link the variables to the constraint since that happens in AddConstraintSet
    constraint->LinkWithVariables(nlp->GetOptVariables());
    auto cost = std::make_shared<trajopt::SquaredCost>(constraint, coeff);
    nlp->AddCostSet(cost);
  }
  return true;
}

ifopt::ConstraintSet::Ptr createJointVelocityConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                                                        const std::vector<trajopt::JointPosition::ConstPtr>& vars,
                                                        const Eigen::VectorXd& /*coeffs*/)

{
  assert(!vars.empty());
  trajopt::JointVelConstraint::Ptr vel_constraint =
      std::make_shared<trajopt::JointVelConstraint>(target, vars, "JointVelocity");
  return vel_constraint;
}

bool addJointVelocityConstraint(std::shared_ptr<ifopt::Problem> nlp,
                                std::vector<trajopt::JointPosition::ConstPtr> vars,
                                const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  if (vars.empty())
    return true;

  Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
  auto vel_constraint = createJointVelocityConstraint(vel_target, vars, coeff);
  nlp->AddConstraintSet(vel_constraint);
  return true;
}

bool addJointVelocitySquaredCost(std::shared_ptr<ifopt::Problem> nlp,
                                 std::vector<trajopt::JointPosition::ConstPtr> vars,
                                 const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  if (vars.empty())
    return true;

  Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
  auto vel_constraint = createJointVelocityConstraint(vel_target, vars, coeff);

  // Must link the variables to the constraint since that happens in AddConstraintSet
  vel_constraint->LinkWithVariables(nlp->GetOptVariables());
  auto vel_cost = std::make_shared<trajopt::SquaredCost>(vel_constraint);
  nlp->AddCostSet(vel_cost);
  return true;
}

}  // namespace tesseract_planning
