/**
 * @file utils.h
 * @brief Planner utility functions.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_UTILS_H
#define TESSERACT_MOTION_PLANNERS_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/types.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/command_language_utils.h>

namespace tesseract_planning
{
/**
 * @brief Inerpolate between two transforms return a vector of Eigen::Isometry transforms.
 * @param start The Start Transform
 * @param stop The Stop/End Transform
 * @param steps The number of step
 * @return A vector of Eigen::Isometry with a length = steps + 1
 */
inline tesseract_common::VectorIsometry3d interpolate(const Eigen::Isometry3d& start,
                                                      const Eigen::Isometry3d& stop,
                                                      int steps)
{
  // Required position change
  Eigen::Vector3d delta_translation = (stop.translation() - start.translation());
  Eigen::Vector3d start_pos = start.translation();
  Eigen::Affine3d stop_prime = start.inverse() * stop;
  Eigen::AngleAxisd delta_rotation(stop_prime.rotation());

  // Step size
  Eigen::Vector3d step = delta_translation / steps;

  // Orientation interpolation
  Eigen::Quaterniond start_q(start.rotation());
  Eigen::Quaterniond stop_q(stop.rotation());
  double slerp_ratio = 1.0 / steps;

  tesseract_common::VectorIsometry3d result;
  Eigen::Vector3d trans;
  Eigen::Quaterniond q;
  Eigen::Isometry3d pose;
  result.reserve(static_cast<size_t>(steps) + 1);
  for (unsigned i = 0; i <= static_cast<unsigned>(steps); ++i)
  {
    trans = start_pos + step * i;
    q = start_q.slerp(slerp_ratio * i, stop_q);
    pose = (Eigen::Translation3d(trans) * q);
    result.push_back(pose);
  }
  return result;
}

/**
 * @brief Inerpolate between two Eigen::VectorXd and return a Matrix
 * @param start The Start State
 * @param stop The Stop/End State
 * @param steps The number of step
 * @return A matrix where columns = steps + 1
 */
inline Eigen::MatrixXd interpolate(const Eigen::Ref<const Eigen::VectorXd>& start,
                                   const Eigen::Ref<const Eigen::VectorXd>& stop,
                                   int steps)
{
  assert(start.size() == stop.size());

  Eigen::MatrixXd result(start.size(), steps + 1);

  for (int i = 0; i < start.size(); ++i)
    result.row(i) = Eigen::VectorXd::LinSpaced(steps + 1, start(i), stop(i));

  return result;
}

/**
 * @brief Inerpolate between two waypoints return a vector of waypoints.
 * @param start The Start Waypoint
 * @param stop The Stop/End Waypoint
 * @param steps The number of step
 * @return A vector of waypoints with a length = steps + 1
 */
inline std::vector<Waypoint> interpolate_waypoint(const Waypoint& start, const Waypoint& stop, int steps)
{
  switch (start.getType())
  {
    case static_cast<int>(WaypointType::CARTESIAN_WAYPOINT):
    {
      const auto* w1 = start.cast_const<Eigen::Isometry3d>();
      const auto* w2 = stop.cast_const<Eigen::Isometry3d>();
      tesseract_common::VectorIsometry3d eigen_poses = interpolate(*w1, *w2, steps);

      std::vector<Waypoint> result;
      result.reserve(eigen_poses.size());
      for (auto& eigen_pose : eigen_poses)
        result.emplace_back(CartesianWaypoint(eigen_pose));

      return result;
    }
    case static_cast<int>(WaypointType::JOINT_WAYPOINT):
    {
      const auto* jwp1 = start.cast_const<JointWaypoint>();
      //      const auto* jwp2 = stop.cast_const<JointWaypoint>();

      // TODO: Should check joint names are in the same order

      const auto* w1 = start.cast_const<Eigen::VectorXd>();
      const auto* w2 = stop.cast_const<Eigen::VectorXd>();
      Eigen::MatrixXd joint_poses = interpolate(*w1, *w2, steps);

      std::vector<Waypoint> result;
      result.reserve(static_cast<std::size_t>(joint_poses.cols()));
      for (int i = 0; i < joint_poses.cols(); ++i)
        result.emplace_back(JointWaypoint(jwp1->joint_names, joint_poses.col(i)));

      return result;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("Interpolator for Waypoint type %d is currently not support!", start.getType());
      return std::vector<Waypoint>();
    }
  }
}

static locateFilterFn moveFilter =
    [](const Instruction& i, const CompositeInstruction& /*composite*/, bool parent_is_first_composite) {
      if (isMoveInstruction(i))
      {
        if (i.cast_const<MoveInstruction>()->isStart())
          return (parent_is_first_composite);

        return true;
      }
      return false;
    };

static locateFilterFn planFilter =
    [](const Instruction& i, const CompositeInstruction& /*composite*/, bool parent_is_first_composite) {
      if (isPlanInstruction(i))
      {
        if (i.cast_const<PlanInstruction>()->isStart())
          return (parent_is_first_composite);

        return true;
      }
      return false;
    };

static flattenFilterFn programFlattenFilter =
    [](const Instruction& i, const CompositeInstruction& /*composite*/, bool parent_is_first_composite) {
      if (isMoveInstruction(i))
      {
        if (i.cast_const<MoveInstruction>()->isStart())
          return (parent_is_first_composite);
      }
      else if (isPlanInstruction(i))
      {
        if (i.cast_const<PlanInstruction>()->isStart())
          return (parent_is_first_composite);
      }
      else if (isCompositeInstruction(i))
      {
        return false;
      }

      return true;
    };

/**
 * @brief Get the first Move Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The first Move Instruction (Non-Const)
 */
inline MoveInstruction* getFirstMoveInstruction(CompositeInstruction& composite_instruction)
{
  Instruction* mi = getFirstInstruction(composite_instruction, moveFilter);
  if (mi)
    return mi->cast<MoveInstruction>();

  return nullptr;
}

/**
 * @brief Get the first Move Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The first Move Instruction (Const)
 */
inline const MoveInstruction* getFirstMoveInstruction(const CompositeInstruction& composite_instruction)
{
  const Instruction* mi = getFirstInstruction(composite_instruction, moveFilter);
  if (mi)
    return mi->cast_const<MoveInstruction>();

  return nullptr;
}

/**
 * @brief Get the first Plan Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The first Plan Instruction (Non-Const)
 */
inline PlanInstruction* getFirstPlanInstruction(CompositeInstruction& composite_instruction)
{
  Instruction* mi = getFirstInstruction(composite_instruction, planFilter);
  if (mi)
    return mi->cast<PlanInstruction>();

  return nullptr;
}

/**
 * @brief Get the first Plan Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The first Plan Instruction (Const)
 */
inline const PlanInstruction* getFirstPlanInstruction(const CompositeInstruction& composite_instruction)
{
  const Instruction* mi = getFirstInstruction(composite_instruction, planFilter);
  if (mi)
    return mi->cast_const<PlanInstruction>();

  return nullptr;
}

/**
 * @brief Get the last Move Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The last Move Instruction (Non-Const)
 */
inline MoveInstruction* getLastMoveInstruction(CompositeInstruction& composite_instruction)
{
  Instruction* mi = getLastInstruction(composite_instruction, moveFilter);
  if (mi)
    return mi->cast<MoveInstruction>();

  return nullptr;
}

/**
 * @brief Get the last Move Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The last Move Instruction (Const)
 */
inline const MoveInstruction* getLastMoveInstruction(const CompositeInstruction& composite_instruction)
{
  const Instruction* mi = getLastInstruction(composite_instruction, moveFilter);
  if (mi)
    return mi->cast_const<MoveInstruction>();

  return nullptr;
}

/**
 * @brief Get the last Plan Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The last Plan Instruction (Non-Const)
 */
inline PlanInstruction* getLastPlanInstruction(CompositeInstruction& composite_instruction)
{
  Instruction* mi = getLastInstruction(composite_instruction, planFilter);
  if (mi)
    return mi->cast<PlanInstruction>();

  return nullptr;
}

/**
 * @brief Get the last Plan Instruction in a Composite Instruction
 * This does not consider the start instruction in child composite instruction
 * @param composite_instruction Composite Instruction to search
 * @return The last Plan Instruction (Const)
 */
inline const PlanInstruction* getLastPlanInstruction(const CompositeInstruction& composite_instruction)
{
  const Instruction* mi = getLastInstruction(composite_instruction, planFilter);
  if (mi)
    return mi->cast_const<PlanInstruction>();

  return nullptr;
}

/**
 * @brief Flattens a CompositeInstruction into a vector of Instruction&
 *
 * If /p composite_instruction parameter has a start instruction it is added but child composites are not check for
 * start instructions.
 *
 * @param composite_instruction Input composite instruction to be flattened
 * @return A new flattened vector referencing the original instruction elements
 */
inline std::vector<std::reference_wrapper<Instruction>> flattenProgram(CompositeInstruction& composite_instruction)
{
  return flatten(composite_instruction, programFlattenFilter);
}

/**
 * @brief Flattens a CompositeInstruction into a vector of Instruction&
 *
 * If /p composite_instruction parameter has a start instruction it is added but child composites are not check for
 * start instructions.
 *
 * @param composite_instruction Input composite instruction to be flattened
 * @return A new flattened vector (const) referencing the original instruction elements
 */
inline std::vector<std::reference_wrapper<const Instruction>>
flattenProgram(const CompositeInstruction& composite_instruction)
{
  return flatten(composite_instruction, programFlattenFilter);
}

/**
 * @brief Flattens a composite instruction to the same pattern as the pattern composite instruction. ie, an element of
 * instruction will only be flattened if the corresponding element in pattern is flattenable.
 *
 * If /p composite_instruction parameter has a start instruction it is added but child composites are not check for
 * start instructions.
 *
 * The motivation for this utility is a case where you flatten only the elements in a seed that correspond to composites
 * in the parent instruction
 *
 * @param instruction CompositeInstruction that will be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @return A new flattened vector referencing the original instruction elements
 */
inline std::vector<std::reference_wrapper<Instruction>>
flattenProgramToPattern(CompositeInstruction& composite_instruction, const CompositeInstruction& pattern)
{
  return flattenToPattern(composite_instruction, pattern, programFlattenFilter);
}

/**
 * @brief Flattens a composite instruction to the same pattern as the pattern composite instruction. ie, an element of
 * instruction will only be flattened if the corresponding element in pattern is flattenable.
 *
 * If /p composite_instruction parameter has a start instruction it is added but child composites are not check for
 * start instructions.
 *
 * The motivation for this utility is a case where you flatten only the elements in a seed that correspond to composites
 * in the parent instruction
 *
 * @param instruction CompositeInstruction that will be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @return A new flattened vector (const) referencing the original instruction elements
 */
inline std::vector<std::reference_wrapper<const Instruction>>
flattenProgramToPattern(const CompositeInstruction& composite_instruction, const CompositeInstruction& pattern)
{
  return flattenToPattern(composite_instruction, pattern, programFlattenFilter);
}

/**
 * @brief Get number of Move Instruction in a Composite Instruction
 * This does not consider the start instruction in the child composite instruction
 * @param composite_instruction The Composite Instruction to process
 * @return The number of Move Instructions
 */
inline long getMoveInstructionCount(const CompositeInstruction& composite_instruction)
{
  return getInstructionCount(composite_instruction, moveFilter);
}

/**
 * @brief Get number of Plan Instruction in a Composite Instruction
 * This does not consider the start instruction in the child composite instruction
 * @param composite_instruction The Composite Instruction to process
 * @return The number of Plan Instructions
 */
inline long getPlanInstructionCount(const CompositeInstruction& composite_instruction)
{
  return getInstructionCount(composite_instruction, planFilter);
}

/**
 * @brief Should perform a continuous collision check over the trajectory.
 * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param program The program to check for collisions
 * @param request Contact request data
 * @param verbose Print out found collisions
 * @return True if collision was found, otherwise false.
 */
inline bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                                tesseract_collision::ContinuousContactManager& manager,
                                const tesseract_environment::StateSolver& state_solver,
                                const CompositeInstruction& program,
                                const tesseract_collision::ContactRequest& request =
                                    tesseract_collision::ContactRequest(tesseract_collision::ContactTestType::FIRST),
                                bool verbose = false)
{
  bool found = false;

  // Flatten results
  std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

  contacts.reserve(mi.size());
  for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
  {
    const auto* swp0 = mi.at(iStep).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();
    const auto* swp1 = mi.at(iStep + 1).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();
    tesseract_environment::EnvState::Ptr state0 = state_solver.getState(swp0->joint_names, swp0->position);
    tesseract_environment::EnvState::Ptr state1 = state_solver.getState(swp1->joint_names, swp1->position);

    if (checkTrajectorySegment(contacts, manager, state0, state1, request, verbose))
    {
      found = true;
      if (verbose)
      {
        std::stringstream ss;
        ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

        ss << "     Names:";
        for (const auto& name : swp0->joint_names)
          ss << " " << name;

        ss << std::endl
           << "    State0: " << swp0->position << std::endl
           << "    State1: " << swp1->position << std::endl;

        CONSOLE_BRIDGE_logError(ss.str().c_str());
      }
    }

    if (found && (request.type == tesseract_collision::ContactTestType::FIRST))
      break;
  }

  return found;
}

/**
 * @brief Should perform a continuous collision check over the trajectory.
 * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param program The program to check for contacts
 * @param longest_valid_segment_length Used to check collisions between two state if norm(state0-state1) >
 * longest_valid_segment_length.
 * @param request Contact request data
 * @param verbose Print out found collisions
 * @return True if collision was found, otherwise false.
 */
inline bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                                tesseract_collision::ContinuousContactManager& manager,
                                const tesseract_environment::StateSolver& state_solver,
                                const CompositeInstruction& program,
                                double longest_valid_segment_length,
                                const tesseract_collision::ContactRequest& request =
                                    tesseract_collision::ContactRequest(tesseract_collision::ContactTestType::FIRST),
                                bool verbose = false)
{
  bool found = false;

  // Flatten results
  std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

  contacts.reserve(mi.size() - 1);
  for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
  {
    const auto* swp0 = mi.at(iStep).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();
    const auto* swp1 = mi.at(iStep + 1).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();

    // TODO: Should check joint names and make sure they are in the same order
    double dist = (swp1->position - swp0->position).norm();
    if (dist > longest_valid_segment_length)
    {
      long cnt = static_cast<long>(std::ceil(dist / longest_valid_segment_length)) + 1;
      tesseract_common::TrajArray subtraj(cnt, swp0->position.size());
      for (long iVar = 0; iVar < swp0->position.size(); ++iVar)
        subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, swp0->position(iVar), swp1->position(iVar));

      for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
      {
        tesseract_environment::EnvState::Ptr state0 = state_solver.getState(swp0->joint_names, subtraj.row(iSubStep));
        tesseract_environment::EnvState::Ptr state1 =
            state_solver.getState(swp0->joint_names, subtraj.row(iSubStep + 1));
        if (checkTrajectorySegment(contacts, manager, state0, state1, request, verbose))
        {
          found = true;
          if (verbose)
          {
            std::stringstream ss;
            ss << "Continuous collision detected at step: " << iStep << " of " << (mi.size() - 1)
               << " substep: " << iSubStep << std::endl;

            ss << "     Names:";
            for (const auto& name : swp0->joint_names)
              ss << " " << name;

            ss << std::endl
               << "    State0: " << subtraj.row(iSubStep) << std::endl
               << "    State1: " << subtraj.row(iSubStep + 1) << std::endl;

            CONSOLE_BRIDGE_logError(ss.str().c_str());
          }
        }

        if (found && (request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }

      if (found && (request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
    else
    {
      tesseract_environment::EnvState::Ptr state0 = state_solver.getState(swp0->joint_names, swp0->position);
      tesseract_environment::EnvState::Ptr state1 = state_solver.getState(swp1->joint_names, swp1->position);
      if (checkTrajectorySegment(contacts, manager, state0, state1, request, verbose))
      {
        found = true;
        if (verbose)
        {
          std::stringstream ss;
          ss << "Continuous collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

          ss << "     Names:";
          for (const auto& name : swp0->joint_names)
            ss << " " << name;

          ss << std::endl
             << "    State0: " << swp0->position << std::endl
             << "    State1: " << swp1->position << std::endl;

          CONSOLE_BRIDGE_logError(ss.str().c_str());
        }
      }

      if (found && (request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }

  return found;
}

/**
 * @brief Should perform a discrete collision check over the trajectory
 * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param program The program to check for contacts
 * @param request Contact request data
 * @param verbose Print out found collisions
 * @return True if collision was found, otherwise false.
 */
inline bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                                tesseract_collision::DiscreteContactManager& manager,
                                const tesseract_environment::StateSolver& state_solver,
                                const CompositeInstruction& program,
                                const tesseract_collision::ContactRequest& request =
                                    tesseract_collision::ContactRequest(tesseract_collision::ContactTestType::FIRST),
                                bool verbose = false)
{
  bool found = false;

  // Flatten results
  std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

  contacts.reserve(mi.size());
  for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
  {
    const auto* swp0 = mi.at(iStep).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();

    tesseract_environment::EnvState::Ptr state = state_solver.getState(swp0->joint_names, swp0->position);
    if (checkTrajectoryState(contacts, manager, state, request, verbose))
    {
      found = true;
      if (verbose)
      {
        std::stringstream ss;
        ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

        ss << "     Names:";
        for (const auto& name : swp0->joint_names)
          ss << " " << name;

        ss << std::endl << "    State0: " << swp0->position << std::endl;

        CONSOLE_BRIDGE_logError(ss.str().c_str());
      }
    }

    if (found && (request.type == tesseract_collision::ContactTestType::FIRST))
      break;
  }

  return found;
}

/**
 * @brief Should perform a discrete collision check over the trajectory
 * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param program The program to check for contacts
 * @param longest_valid_segment_length Used to check collisions between two state if norm(state0-state1) >
 * longest_valid_segment_length.
 * @param request Contact request data
 * @param verbose Print out found collisions
 * @return True if collision was found, otherwise false.
 */
inline bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                                tesseract_collision::DiscreteContactManager& manager,
                                const tesseract_environment::StateSolver& state_solver,
                                const CompositeInstruction& program,
                                double longest_valid_segment_length,
                                const tesseract_collision::ContactRequest& request =
                                    tesseract_collision::ContactRequest(tesseract_collision::ContactTestType::FIRST),
                                bool verbose = false)
{
  bool found = false;

  // Flatten results
  std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

  contacts.reserve(mi.size());
  for (std::size_t iStep = 0; iStep < mi.size(); ++iStep)
  {
    const auto* swp0 = mi.at(iStep).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();
    const auto* swp1 = mi.at(iStep + 1).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();

    double dist = -1;
    if (iStep < mi.size() - 1)
      dist = (swp1->position - swp0->position).norm();

    if (dist > 0 && dist > longest_valid_segment_length)
    {
      int cnt = static_cast<int>(std::ceil(dist / longest_valid_segment_length)) + 1;
      tesseract_common::TrajArray subtraj(cnt, swp0->position.size());
      for (long iVar = 0; iVar < swp0->position.size(); ++iVar)
        subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, swp0->position(iVar), swp1->position(iVar));

      for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
      {
        tesseract_environment::EnvState::Ptr state = state_solver.getState(swp0->joint_names, subtraj.row(iSubStep));
        if (checkTrajectoryState(contacts, manager, state, request, verbose))
        {
          found = true;
          if (verbose)
          {
            std::stringstream ss;
            ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1)
               << " substate: " << iSubStep << std::endl;

            ss << "     Names:";
            for (const auto& name : swp0->joint_names)
              ss << " " << name;

            ss << std::endl << "    State: " << subtraj.row(iSubStep) << std::endl;

            CONSOLE_BRIDGE_logError(ss.str().c_str());
          }
        }

        if (found && (request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }

      if (found && (request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
    else
    {
      const auto* swp0 = mi.at(iStep).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();
      tesseract_environment::EnvState::Ptr state = state_solver.getState(swp0->joint_names, swp0->position);
      if (checkTrajectoryState(contacts, manager, state, request, verbose))
      {
        found = true;
        if (verbose)
        {
          std::stringstream ss;
          ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

          ss << "     Names:";
          for (const auto& name : swp0->joint_names)
            ss << " " << name;

          ss << std::endl << "    State: " << swp0->position << std::endl;

          CONSOLE_BRIDGE_logError(ss.str().c_str());
        }
      }

      if (found && (request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }

  return found;
}

}  // namespace tesseract_planning

#endif  // TESSERACT_PLANNING_UTILS_H
