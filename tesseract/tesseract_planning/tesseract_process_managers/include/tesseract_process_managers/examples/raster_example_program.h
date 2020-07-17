﻿#ifndef TESSERACT_PROCESS_MANAGERS_EXAMPLE_PROGRAM_H
#define TESSERACT_PROCESS_MANAGERS_EXAMPLE_PROGRAM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/instruction.h>

#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
inline CompositeInstruction rasterExampleProgram()
{
  CompositeInstruction program;
  // Start Joint Position for the program
  Waypoint wp1 = JointWaypoint(Eigen::VectorXd::Ones(6));
  PlanInstruction start_instruction(wp1, PlanInstructionType::START_FIXED);
  program.setStartInstruction(start_instruction);

  // Define raster poses
  Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -1, 1));
  Waypoint wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -0.4, 1));
  Waypoint wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -0.2, 1));
  Waypoint wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.0, 1));
  Waypoint wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.2, 1));
  Waypoint wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.4, 1));

  // Define raster move instruction
  PlanInstruction plan_c1(wp3, PlanInstructionType::LINEAR, "cartesian_profile");
  PlanInstruction plan_c2(wp4, PlanInstructionType::LINEAR, "cartesian_profile");
  PlanInstruction plan_c3(wp5, PlanInstructionType::LINEAR, "cartesian_profile");
  PlanInstruction plan_c4(wp6, PlanInstructionType::LINEAR, "cartesian_profile");
  PlanInstruction plan_c5(wp7, PlanInstructionType::LINEAR, "cartesian_profile");

  PlanInstruction plan_f0(wp2, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f0.setDescription("from_start_plan");
  CompositeInstruction from_start;
  from_start.setDescription("from_start");
  from_start.push_back(plan_f0);
  program.push_back(from_start);

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    CompositeInstruction raster;
    raster.setDescription("raster");
    raster.push_back(plan_c1);
    raster.push_back(plan_c2);
    raster.push_back(plan_c3);
    raster.push_back(plan_c4);
    raster.push_back(plan_c5);
    raster_segment.push_back(raster);
    program.push_back(raster_segment);
  }

  {
    PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, "freespace_profile");
    plan_f1.setDescription("transition_from_end_plan");
    CompositeInstruction transition_from_end;
    transition_from_end.setDescription("transition_from_end");
    transition_from_end.push_back(plan_f1);
    CompositeInstruction transition_from_start;
    transition_from_start.setDescription("transition_from_start");
    transition_from_start.push_back(plan_f1);

    CompositeInstruction transitions("DEFAULT", CompositeInstructionOrder::UNORDERED);
    transitions.setDescription("transitions");
    transitions.push_back(transition_from_start);
    transitions.push_back(transition_from_end);
    program.push_back(transitions);
  }

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    CompositeInstruction raster;
    raster.setDescription("raster");
    raster.push_back(plan_c1);
    raster.push_back(plan_c2);
    raster.push_back(plan_c3);
    raster.push_back(plan_c4);
    raster.push_back(plan_c5);
    raster_segment.push_back(raster);
    program.push_back(raster_segment);
  }

  {
    PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, "freespace_profile");
    plan_f1.setDescription("transition_from_end_plan");
    CompositeInstruction transition_from_end;
    transition_from_end.setDescription("transition_from_end");
    transition_from_end.push_back(plan_f1);
    CompositeInstruction transition_from_start;
    transition_from_start.setDescription("transition_from_start");
    transition_from_start.push_back(plan_f1);

    CompositeInstruction transitions("DEFAULT", CompositeInstructionOrder::UNORDERED);
    transitions.setDescription("transitions");
    transitions.push_back(transition_from_start);
    transitions.push_back(transition_from_end);
    program.push_back(transitions);
  }

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    CompositeInstruction raster;
    raster.setDescription("raster");
    raster.push_back(plan_c1);
    raster.push_back(plan_c2);
    raster.push_back(plan_c3);
    raster.push_back(plan_c4);
    raster.push_back(plan_c5);
    raster_segment.push_back(raster);
    program.push_back(raster_segment);
  }

  PlanInstruction plan_f2(wp2, PlanInstructionType::FREESPACE, "freespace_profile");
  plan_f2.setDescription("to_end_plan");
  CompositeInstruction to_end;
  to_end.setDescription("to_end");
  to_end.push_back(plan_f2);
  program.push_back(to_end);

  return program;
}

}  // namespace tesseract_planning

#endif