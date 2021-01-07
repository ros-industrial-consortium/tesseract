/**
 * @file default_problem_generator.cpp
 * @brief Generates a trajopt problem from a planner request
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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

#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>

namespace tesseract_planning
{
/// @todo: Restructure this into several smaller functions that are testable and easier to understand
std::shared_ptr<trajopt::ProblemConstructionInfo>
DefaultTrajoptProblemGenerator(const std::string& name,
                               const PlannerRequest& request,
                               const TrajOptPlanProfileMap& plan_profiles,
                               const TrajOptCompositeProfileMap& composite_profiles,
                               const TrajOptSolverProfileMap& solver_profiles)
{
  auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(request.env);

  // Store fixed steps
  std::vector<int> fixed_steps;

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!request.instructions.getManipulatorInfo().empty());
  const ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();

  // Assign Kinematics object
  pci->kin = pci->getManipulator(composite_mi.manipulator);
  std::string link = pci->kin->getTipLinkName();

  if (pci->kin == nullptr)
  {
    std::string error_msg = "In trajopt_array_planner: manipulator_ does not exist in kin_map_";
    CONSOLE_BRIDGE_logError(error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  // Apply Solver parameters
  std::string profile = request.instructions.getProfile();
  profile = getProfileString(profile, name, PlannerProfileRemapping());
  TrajOptSolverProfile::ConstPtr solver_profile =
      getProfile<TrajOptSolverProfile>(profile, solver_profiles, std::make_shared<TrajOptDefaultSolverProfile>());
  if (!solver_profile)
    throw std::runtime_error("TrajOptSolverConfig: Invalid profile");

  solver_profile->apply(*pci);

  // Flatten the input for planning
  auto instructions_flat = flattenProgram(request.instructions);
  auto seed_flat = flattenProgramToPattern(request.seed, request.instructions);

  // Get kinematics information
  tesseract_environment::Environment::ConstPtr env = request.env;
  tesseract_environment::AdjacencyMap map(
      env->getSceneGraph(), pci->kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);
  const std::vector<std::string>& active_links = map.getActiveLinkNames();

  // Create a temp seed storage.
  std::vector<Eigen::VectorXd> seed_states;
  seed_states.reserve(instructions_flat.size());

  std::size_t start_index = 0;  // If it has a start instruction then skip first instruction in instructions_flat
  int index = 0;
  Waypoint start_waypoint = NullWaypoint();
  Instruction placeholder_instruction = NullInstruction();
  const Instruction* start_instruction = nullptr;
  if (request.instructions.hasStartInstruction())
  {
    assert(isPlanInstruction(request.instructions.getStartInstruction()));
    start_instruction = &(request.instructions.getStartInstruction());
    if (isPlanInstruction(*start_instruction))
    {
      const auto* temp = start_instruction->cast_const<PlanInstruction>();
      assert(temp->isStart());
      start_waypoint = temp->getWaypoint();
      profile = temp->getProfile();
    }
    else
    {
      throw std::runtime_error("TrajOpt DefaultProblemGenerator: Unsupported start instruction type!");
    }
    ++start_index;
  }
  else
  {
    Eigen::VectorXd current_jv = request.env_state->getJointValues(pci->kin->getJointNames());
    StateWaypoint swp(pci->kin->getJointNames(), current_jv);

    MoveInstruction temp_move(swp, MoveInstructionType::START);
    placeholder_instruction = temp_move;
    start_instruction = &placeholder_instruction;
    start_waypoint = swp;
  }

  profile = getProfileString(profile, name, request.plan_profile_remapping);
  TrajOptPlanProfile::ConstPtr start_plan_profile =
      getProfile<TrajOptPlanProfile>(profile, plan_profiles, std::make_shared<TrajOptDefaultPlanProfile>());
  if (!start_plan_profile)
    throw std::runtime_error("TrajOptPlannerUniversalConfig: Invalid profile");

  // Add start seed state
  assert(request.seed.hasStartInstruction());
  assert(isMoveInstruction(request.seed.getStartInstruction()));
  const auto* seed_instruction = request.seed.getStartInstruction().cast_const<MoveInstruction>();
  assert(checkJointPositionFormat(pci->kin->getJointNames(), seed_instruction->getWaypoint()));
  seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

  // Add start waypoint
  if (isCartesianWaypoint(start_waypoint))
  {
    const auto* cwp = start_waypoint.cast_const<Eigen::Isometry3d>();
    start_plan_profile->apply(*pci, *cwp, *start_instruction, composite_mi, active_links, index);
  }
  else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
  {
    assert(checkJointPositionFormat(pci->kin->getJointNames(), start_waypoint));
    JointWaypoint position_wp;
    position_wp.waypoint = getJointPosition(start_waypoint);
    start_plan_profile->apply(*pci, position_wp, *start_instruction, composite_mi, active_links, index);

    // Add to fixed indices
    fixed_steps.push_back(index);
  }
  else
  {
    throw std::runtime_error("TrajOpt Problem Generator: unknown waypoint type.");
  }

  ++index;

  // ----------------
  // Translate TCL
  // ----------------

  // Transform plan instructions into trajopt cost and constraints
  for (std::size_t i = start_index; i < instructions_flat.size(); ++i)
  {
    const auto& instruction = instructions_flat[i].get();
    if (isPlanInstruction(instruction))
    {
      assert(isPlanInstruction(instruction));
      const auto* plan_instruction = instruction.cast_const<PlanInstruction>();

      // If plan instruction has manipulator information then use it over the one provided by the composite.
      ManipulatorInfo manip_info = composite_mi.getCombined(plan_instruction->getManipulatorInfo());
      Eigen::Isometry3d tcp = request.env->findTCP(manip_info);

      assert(isCompositeInstruction(seed_flat[i].get()));
      const auto* seed_composite = seed_flat[i].get().cast_const<tesseract_planning::CompositeInstruction>();
      auto interpolate_cnt = static_cast<int>(seed_composite->size());

      std::string profile = getProfileString(plan_instruction->getProfile(), name, request.plan_profile_remapping);
      TrajOptPlanProfile::ConstPtr cur_plan_profile =
          getProfile<TrajOptPlanProfile>(profile, plan_profiles, std::make_shared<TrajOptDefaultPlanProfile>());
      if (!cur_plan_profile)
        throw std::runtime_error("TrajOptPlannerUniversalConfig: Invalid profile");

      if (plan_instruction->isLinear())
      {
        if (isCartesianWaypoint(plan_instruction->getWaypoint()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (isCartesianWaypoint(start_waypoint))
          {
            prev_pose = *(start_waypoint.cast_const<Eigen::Isometry3d>());
          }
          else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
          {
            assert(checkJointPositionFormat(pci->kin->getJointNames(), start_waypoint));
            const Eigen::VectorXd& position = getJointPosition(start_waypoint);
            if (!pci->kin->calcFwdKin(prev_pose, position))
              throw std::runtime_error("TrajOptPlannerUniversalConfig: failed to solve forward kinematics!");

            prev_pose = pci->env->getCurrentState()->link_transforms.at(pci->kin->getBaseLinkName()) * prev_pose * tcp;
          }
          else
          {
            throw std::runtime_error("TrajOptPlannerUniversalConfig: uknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, *cur_wp, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_plan_profile->apply(*pci, poses[p], *plan_instruction, composite_mi, active_links, index);

            // Add seed state
            assert(isMoveInstruction(seed_composite->at(p)));
            const auto* seed_instruction = seed_composite->at(p).cast_const<MoveInstruction>();
            assert(checkJointPositionFormat(pci->kin->getJointNames(), seed_instruction->getWaypoint()));
            seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*pci, *cur_wp, *plan_instruction, composite_mi, active_links, index);

          // Add seed state
          assert(isMoveInstruction(seed_composite->back()));
          const auto* seed_instruction = seed_composite->back().cast_const<tesseract_planning::MoveInstruction>();
          assert(checkJointPositionFormat(pci->kin->getJointNames(), seed_instruction->getWaypoint()));
          seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

          ++index;
        }
        else if (isJointWaypoint(plan_instruction->getWaypoint()) || isStateWaypoint(plan_instruction->getWaypoint()))
        {
          assert(checkJointPositionFormat(pci->kin->getJointNames(), plan_instruction->getWaypoint()));
          JointWaypoint cur_position;
          cur_position.waypoint = getJointPosition(plan_instruction->getWaypoint());
          Eigen::Isometry3d cur_pose = Eigen::Isometry3d::Identity();
          if (!pci->kin->calcFwdKin(cur_pose, cur_position))
            throw std::runtime_error("TrajOptPlannerUniversalConfig: failed to solve forward kinematics!");

          cur_pose = pci->env->getCurrentState()->link_transforms.at(pci->kin->getBaseLinkName()) * cur_pose * tcp;

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (isCartesianWaypoint(start_waypoint))
          {
            prev_pose = *(start_waypoint.cast_const<Eigen::Isometry3d>());
          }
          else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
          {
            assert(checkJointPositionFormat(pci->kin->getJointNames(), start_waypoint));
            const Eigen::VectorXd& position = getJointPosition(start_waypoint);
            if (!pci->kin->calcFwdKin(prev_pose, position))
              throw std::runtime_error("TrajOptPlannerUniversalConfig: failed to solve forward kinematics!");

            prev_pose = pci->env->getCurrentState()->link_transforms.at(pci->kin->getBaseLinkName()) * prev_pose * tcp;
          }
          else
          {
            throw std::runtime_error("TrajOptPlannerUniversalConfig: uknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_pose, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_plan_profile->apply(*pci, poses[p], *plan_instruction, composite_mi, active_links, index);

            // Add seed state
            assert(isMoveInstruction(seed_composite->at(p)));
            const auto* seed_instruction = seed_composite->at(p).cast_const<MoveInstruction>();
            assert(checkJointPositionFormat(pci->kin->getJointNames(), seed_instruction->getWaypoint()));
            seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*pci, cur_position, *plan_instruction, composite_mi, active_links, index);

          // Add to fixed indices
          fixed_steps.push_back(index);

          // Add seed state
          assert(isMoveInstruction(seed_composite->back()));
          const auto* seed_instruction = seed_composite->back().cast_const<MoveInstruction>();
          assert(checkJointPositionFormat(pci->kin->getJointNames(), seed_instruction->getWaypoint()));
          seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

          ++index;
        }
        else
        {
          throw std::runtime_error("TrajOptPlannerUniversalConfig: unknown waypoint type");
        }
      }
      else if (plan_instruction->isFreespace())
      {
        if (isJointWaypoint(plan_instruction->getWaypoint()) || isStateWaypoint(plan_instruction->getWaypoint()))
        {
          assert(checkJointPositionFormat(pci->kin->getJointNames(), plan_instruction->getWaypoint()));
          JointWaypoint cur_position;
          cur_position.waypoint = getJointPosition(plan_instruction->getWaypoint());

          // Add intermediate points with path costs and constraints
          for (std::size_t s = 0; s < seed_composite->size() - 1; ++s)
          {
            // Add seed state
            assert(isMoveInstruction(seed_composite->at(s)));
            const auto* seed_instruction = seed_composite->at(s).cast_const<MoveInstruction>();
            assert(checkJointPositionFormat(pci->kin->getJointNames(), seed_instruction->getWaypoint()));
            seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

            ++index;
          }

          // Add final point with waypoint costs and contraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(*pci, cur_position, *plan_instruction, composite_mi, active_links, index);

          // Add to fixed indices
          fixed_steps.push_back(index);

          // Add seed state
          assert(isMoveInstruction(seed_composite->back()));
          const auto* seed_instruction = seed_composite->back().cast_const<MoveInstruction>();
          assert(checkJointPositionFormat(pci->kin->getJointNames(), seed_instruction->getWaypoint()));
          seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));
        }
        else if (isCartesianWaypoint(plan_instruction->getWaypoint()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<CartesianWaypoint>();

          // Add intermediate points with path costs and constraints
          for (std::size_t s = 0; s < seed_composite->size() - 1; ++s)
          {
            // Add seed state
            assert(isMoveInstruction(seed_composite->at(s)));
            const auto* seed_instruction = seed_composite->at(s).cast_const<MoveInstruction>();
            assert(checkJointPositionFormat(pci->kin->getJointNames(), seed_instruction->getWaypoint()));
            seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

            ++index;
          }

          // Add final point with waypoint costs and contraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(*pci, *cur_wp, *plan_instruction, composite_mi, active_links, index);

          // Add seed state
          assert(isMoveInstruction(seed_composite->back()));
          const auto* seed_instruction = seed_composite->back().cast_const<MoveInstruction>();
          assert(checkJointPositionFormat(pci->kin->getJointNames(), seed_instruction->getWaypoint()));
          seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));
        }
        else
        {
          throw std::runtime_error("TrajOptPlannerUniversalConfig: unknown waypoint type");
        }

        ++index;
      }
      else
      {
        throw std::runtime_error("Unsupported!");
      }

      start_waypoint = plan_instruction->getWaypoint();
    }
  }

  // ----------------
  // Create Problem
  // ----------------

  // Setup Basic Info
  pci->basic_info.n_steps = index;
  pci->basic_info.manip = composite_mi.manipulator;
  pci->basic_info.use_time = false;

  // Add the fixed timesteps. TrajOpt will constrain the optimization such that any costs applied at these timesteps
  // will be ignored. Costs applied to variables at fixed timesteps generally causes solver failures
  pci->basic_info.fixed_timesteps = fixed_steps;

  // Set trajopt seed
  assert(static_cast<long>(seed_states.size()) == pci->basic_info.n_steps);
  pci->init_info.type = trajopt::InitInfo::GIVEN_TRAJ;
  pci->init_info.data.resize(pci->basic_info.n_steps, pci->kin->numJoints());
  for (long i = 0; i < pci->basic_info.n_steps; ++i)
    pci->init_info.data.row(i) = seed_states[static_cast<std::size_t>(i)];

  profile = getProfileString(request.instructions.getProfile(), name, request.composite_profile_remapping);
  TrajOptCompositeProfile::ConstPtr cur_composite_profile = getProfile<TrajOptCompositeProfile>(
      profile, composite_profiles, std::make_shared<TrajOptDefaultCompositeProfile>());
  if (!cur_composite_profile)
    throw std::runtime_error("TrajOptPlannerUniversalConfig: Invalid profile");

  cur_composite_profile->apply(*pci, 0, pci->basic_info.n_steps - 1, composite_mi, active_links, fixed_steps);

  return pci;
}

}  // namespace tesseract_planning
