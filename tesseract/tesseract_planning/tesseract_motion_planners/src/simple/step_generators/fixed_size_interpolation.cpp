/**
 * @file fixed_size_interpolation.h
 * @brief
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date July 23, 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/simple/step_generators/fixed_size_interpolation.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/state_waypoint.h>

namespace tesseract_planning
{
CompositeInstruction fixedSizeJointInterpolation(const JointWaypoint& start,
                                                 const JointWaypoint& end,
                                                 const PlanInstruction& base_instruction,
                                                 const PlannerRequest& /*request*/,
                                                 const ManipulatorInfo& /*manip_info*/,
                                                 int steps)
{
  CompositeInstruction composite;

  // Joint waypoints should have joint names
  assert(static_cast<long>(start.joint_names.size()) == start.size());
  assert(static_cast<long>(end.joint_names.size()) == end.size());

  // Linearly interpolate in joint space
  Eigen::MatrixXd states = interpolate(start, end, steps);

  // Convert to MoveInstructions
  for (long i = 1; i < states.cols(); ++i)
  {
    MoveInstruction move_instruction(StateWaypoint(start.joint_names, states.col(i)), MoveInstructionType::FREESPACE);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }
  return composite;
}

CompositeInstruction fixedSizeJointInterpolation(const JointWaypoint& start,
                                                 const CartesianWaypoint& end,
                                                 const PlanInstruction& base_instruction,
                                                 const PlannerRequest& request,
                                                 const ManipulatorInfo& manip_info,
                                                 int steps)
{
  assert(!(manip_info.isEmpty() && base_instruction.getManipulatorInfo().isEmpty()));
  const ManipulatorInfo& mi =
      (base_instruction.getManipulatorInfo().isEmpty()) ? manip_info : base_instruction.getManipulatorInfo();

  // Joint waypoints should have joint names
  assert(static_cast<long>(start.joint_names.size()) == start.size());

  // Initialize
  auto inv_kin = request.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(mi.manipulator);
  auto world_to_base = request.env_state->link_transforms.at(inv_kin->getBaseLinkName());
  const Eigen::Isometry3d& tcp = mi.tcp;
  assert(start.joint_names.size() == inv_kin->getJointNames().size());

  CompositeInstruction composite;

  // Calculate IK for start and end
  Eigen::VectorXd j1 = start;

  Eigen::Isometry3d p2 = end * tcp.inverse();
  p2 = world_to_base.inverse() * p2;
  Eigen::VectorXd j2, j2_final;
  if (!inv_kin->calcInvKin(j2, p2, j1))
    throw std::runtime_error("fixedSizeJointInterpolation: failed to find inverse kinematics solution!");

  // Find closest solution to the start state
  double dist = std::numeric_limits<double>::max();
  const auto dof = inv_kin->numJoints();
  long num_solutions = j2.size() / dof;
  j2_final = j2.middleRows(0, dof);
  for (long i = 0; i < num_solutions; ++i)
  {
    /// @todo: May be nice to add contact checking to find best solution, but may not be neccessary because this is used
    /// to generate the seed.
    auto solution = j2.middleRows(i * dof, dof);
    double d = (solution - j1).norm();
    if (d < dist)
    {
      j2_final = solution;
      dist = d;
    }
  }

  // Linearly interpolate in joint space
  Eigen::MatrixXd states = interpolate(j1, j2_final, steps);

  // Convert to MoveInstructions
  for (long i = 1; i < states.cols(); ++i)
  {
    MoveInstruction move_instruction(StateWaypoint(start.joint_names, states.col(i)), MoveInstructionType::FREESPACE);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }
  return composite;
}

CompositeInstruction fixedSizeJointInterpolation(const CartesianWaypoint& start,
                                                 const JointWaypoint& end,
                                                 const PlanInstruction& base_instruction,
                                                 const PlannerRequest& request,
                                                 const ManipulatorInfo& manip_info,
                                                 int steps)
{
  assert(!(manip_info.isEmpty() && base_instruction.getManipulatorInfo().isEmpty()));
  const ManipulatorInfo& mi =
      (base_instruction.getManipulatorInfo().isEmpty()) ? manip_info : base_instruction.getManipulatorInfo();

  // Joint waypoints should have joint names
  assert(static_cast<long>(end.joint_names.size()) == end.size());

  // Initialize
  auto inv_kin = request.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(mi.manipulator);
  auto world_to_base = request.env_state->link_transforms.at(inv_kin->getBaseLinkName());
  const Eigen::Isometry3d& tcp = mi.tcp;
  assert(end.joint_names.size() == inv_kin->getJointNames().size());

  CompositeInstruction composite;

  // Calculate IK for start and end
  Eigen::Isometry3d p1 = start * tcp.inverse();
  p1 = world_to_base.inverse() * p1;
  Eigen::VectorXd j1, j1_final;
  if (!inv_kin->calcInvKin(j1, p1, end))
    throw std::runtime_error("fixedSizeJointInterpolation: failed to find inverse kinematics solution!");

  Eigen::VectorXd j2 = end;

  // Find closest solution to the end state
  double dist = std::numeric_limits<double>::max();
  const auto dof = inv_kin->numJoints();
  long num_solutions = j1.size() / dof;
  j1_final = j1.middleRows(0, dof);
  for (long i = 0; i < num_solutions; ++i)
  {
    /// @todo: May be nice to add contact checking to find best solution, but may not be neccessary because this is used
    /// to generate the seed.
    auto solution = j1.middleRows(i * dof, dof);
    double d = (j2 - solution).norm();
    if (d < dist)
    {
      j1_final = solution;
      dist = d;
    }
  }

  // Linearly interpolate in joint space
  Eigen::MatrixXd states = interpolate(j1_final, j2, steps);

  // Convert to MoveInstructions
  for (long i = 1; i < states.cols(); ++i)
  {
    MoveInstruction move_instruction(StateWaypoint(end.joint_names, states.col(i)), MoveInstructionType::FREESPACE);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }

  return composite;
}

CompositeInstruction fixedSizeJointInterpolation(const CartesianWaypoint& start,
                                                 const CartesianWaypoint& end,
                                                 const PlanInstruction& base_instruction,
                                                 const PlannerRequest& request,
                                                 const ManipulatorInfo& manip_info,
                                                 int steps)
{
  assert(!(manip_info.isEmpty() && base_instruction.getManipulatorInfo().isEmpty()));
  const ManipulatorInfo& mi =
      (base_instruction.getManipulatorInfo().isEmpty()) ? manip_info : base_instruction.getManipulatorInfo();

  // Initialize
  auto inv_kin = request.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(mi.manipulator);
  auto world_to_base = request.env_state->link_transforms.at(inv_kin->getBaseLinkName());
  const Eigen::Isometry3d& tcp = mi.tcp;

  CompositeInstruction composite;

  // Get IK seed
  Eigen::VectorXd seed = request.env_state->getJointValues(inv_kin->getJointNames());

  // Calculate IK for start and end
  Eigen::Isometry3d p1 = start * tcp.inverse();
  p1 = world_to_base.inverse() * p1;
  Eigen::VectorXd j1, j1_final;
  if (!inv_kin->calcInvKin(j1, p1, seed))
    throw std::runtime_error("fixedSizeJointInterpolation: failed to find inverse kinematics solution!");

  Eigen::Isometry3d p2 = end * tcp.inverse();
  p2 = world_to_base.inverse() * p2;
  Eigen::VectorXd j2, j2_final;
  if (!inv_kin->calcInvKin(j2, p2, seed))
    throw std::runtime_error("fixedSizeJointInterpolation: failed to find inverse kinematics solution!");

  // Find closest solution to the end state
  double dist = std::numeric_limits<double>::max();
  const auto dof = inv_kin->numJoints();
  long j1_num_solutions = j1.size() / dof;
  long j2_num_solutions = j2.size() / dof;
  j1_final = j1.middleRows(0, dof);
  j2_final = j2.middleRows(0, dof);
  for (long i = 0; i < j1_num_solutions; ++i)
  {
    auto j1_solution = j1.middleRows(i * dof, dof);
    for (long j = 0; j < j2_num_solutions; ++j)
    {
      /// @todo: May be nice to add contact checking to find best solution, but may not be neccessary because this is
      /// used to generate the seed.
      auto j2_solution = j2.middleRows(j * dof, dof);
      double d = (j2 - j1).norm();
      if (d < dist)
      {
        j1_final = j1_solution;
        j2_final = j2_solution;
        dist = d;
      }
    }
  }

  // Linearly interpolate in joint space
  Eigen::MatrixXd states = interpolate(j1_final, j2_final, steps);

  // Convert to MoveInstructions
  for (long i = 1; i < states.cols(); ++i)
  {
    MoveInstruction move_instruction(StateWaypoint(inv_kin->getJointNames(), states.col(i)),
                                     MoveInstructionType::FREESPACE);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }

  return composite;
}

CompositeInstruction fixedSizeCartesianInterpolation(const JointWaypoint& start,
                                                     const JointWaypoint& end,
                                                     const PlanInstruction& base_instruction,
                                                     const PlannerRequest& request,
                                                     const ManipulatorInfo& manip_info,
                                                     int steps)
{
  /// @todo: Need to create a cartesian state waypoint and update the code below
  throw std::runtime_error("Not implemented, PR's are welcome!");

  assert(!(manip_info.isEmpty() && base_instruction.getManipulatorInfo().isEmpty()));
  const ManipulatorInfo& mi =
      (base_instruction.getManipulatorInfo().isEmpty()) ? manip_info : base_instruction.getManipulatorInfo();

  // Initialize
  auto fwd_kin = request.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(mi.manipulator);
  auto world_to_base = request.env_state->link_transforms.at(fwd_kin->getBaseLinkName());
  const Eigen::Isometry3d& tcp = mi.tcp;

  CompositeInstruction composite;

  // Calculate FK for start and end
  Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
  if (!fwd_kin->calcFwdKin(p1, start))
    throw std::runtime_error("fixedSizeLinearInterpolation: failed to find forward kinematics solution!");
  p1 = world_to_base * p1 * tcp;

  Eigen::Isometry3d p2 = Eigen::Isometry3d::Identity();
  if (!fwd_kin->calcFwdKin(p2, end))
    throw std::runtime_error("fixedSizeLinearInterpolation: failed to find forward kinematics solution!");
  p2 = world_to_base * p2 * tcp;

  // Linear interpolation in cartesian space
  tesseract_common::VectorIsometry3d poses = interpolate(p1, p2, steps);

  // Convert to MoveInstructions
  for (std::size_t p = 1; p < poses.size(); ++p)
  {
    tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]), MoveInstructionType::LINEAR);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }

  return composite;
}

CompositeInstruction fixedSizeCartesianInterpolation(const JointWaypoint& start,
                                                     const CartesianWaypoint& end,
                                                     const PlanInstruction& base_instruction,
                                                     const PlannerRequest& request,
                                                     const ManipulatorInfo& manip_info,
                                                     int steps)
{
  /// @todo: Need to create a cartesian state waypoint and update the code below
  throw std::runtime_error("Not implemented, PR's are welcome!");

  assert(!(manip_info.isEmpty() && base_instruction.getManipulatorInfo().isEmpty()));
  const ManipulatorInfo& mi =
      (base_instruction.getManipulatorInfo().isEmpty()) ? manip_info : base_instruction.getManipulatorInfo();

  // Initialize
  auto fwd_kin = request.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(mi.manipulator);
  auto world_to_base = request.env_state->link_transforms.at(fwd_kin->getBaseLinkName());
  const Eigen::Isometry3d& tcp = mi.tcp;

  CompositeInstruction composite;

  // Calculate FK for start and end
  Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
  if (!fwd_kin->calcFwdKin(p1, start))
    throw std::runtime_error("fixedSizeLinearInterpolation: failed to find forward kinematics solution!");
  p1 = world_to_base * p1 * tcp;

  Eigen::Isometry3d p2 = end;

  // Linear interpolation in cartesian space
  tesseract_common::VectorIsometry3d poses = interpolate(p1, p2, steps);

  // Convert to MoveInstructions
  for (std::size_t p = 1; p < poses.size(); ++p)
  {
    tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]), MoveInstructionType::LINEAR);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }

  return composite;
}

CompositeInstruction fixedSizeCartesianInterpolation(const CartesianWaypoint& start,
                                                     const JointWaypoint& end,
                                                     const PlanInstruction& base_instruction,
                                                     const PlannerRequest& request,
                                                     const ManipulatorInfo& manip_info,
                                                     int steps)
{
  /// @todo: Need to create a cartesian state waypoint and update the code below
  throw std::runtime_error("Not implemented, PR's are welcome!");

  assert(!(manip_info.isEmpty() && base_instruction.getManipulatorInfo().isEmpty()));
  const ManipulatorInfo& mi =
      (base_instruction.getManipulatorInfo().isEmpty()) ? manip_info : base_instruction.getManipulatorInfo();

  // Initialize
  auto fwd_kin = request.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(mi.manipulator);
  auto world_to_base = request.env_state->link_transforms.at(fwd_kin->getBaseLinkName());
  const Eigen::Isometry3d& tcp = mi.tcp;

  CompositeInstruction composite;

  // Calculate FK for start and end
  Eigen::Isometry3d p1 = start;

  Eigen::Isometry3d p2 = Eigen::Isometry3d::Identity();
  if (!fwd_kin->calcFwdKin(p2, end))
    throw std::runtime_error("fixedSizeLinearInterpolation: failed to find forward kinematics solution!");
  p2 = world_to_base * p2 * tcp;

  // Linear interpolation in cartesian space
  tesseract_common::VectorIsometry3d poses = interpolate(p1, p2, steps);

  // Convert to MoveInstructions
  for (std::size_t p = 1; p < poses.size(); ++p)
  {
    tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]), MoveInstructionType::LINEAR);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }

  return composite;
}

CompositeInstruction fixedSizeCartesianInterpolation(const CartesianWaypoint& start,
                                                     const CartesianWaypoint& end,
                                                     const PlanInstruction& base_instruction,
                                                     const PlannerRequest& /*request*/,
                                                     const ManipulatorInfo& /*manip_info*/,
                                                     int steps)
{
  /// @todo: Need to create a cartesian state waypoint and update the code below
  throw std::runtime_error("Not implemented, PR's are welcome!");

  CompositeInstruction composite;

  // Linear interpolation in cartesian space
  tesseract_common::VectorIsometry3d poses = interpolate(start, end, steps);

  // Convert to MoveInstructions
  for (std::size_t p = 1; p < poses.size(); ++p)
  {
    tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]), MoveInstructionType::LINEAR);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }
  return composite;
}

}  // namespace tesseract_planning
