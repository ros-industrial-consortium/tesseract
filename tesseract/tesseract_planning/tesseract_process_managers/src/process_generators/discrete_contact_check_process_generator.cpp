/**
 * @file discrete_contact_check_process_generator.cpp
 * @brief Discrete collision check trajectory
 *
 * @author Levi Armstrong
 * @date August 10. 2020
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
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_process_managers/process_generators/discrete_contact_check_process_generator.h>

namespace tesseract_planning
{
DiscreteContactCheckProcessGenerator::DiscreteContactCheckProcessGenerator(std::string name) : name_(std::move(name)) {}

DiscreteContactCheckProcessGenerator::DiscreteContactCheckProcessGenerator(double longest_valid_segment_length,
                                                                           double contact_distance,
                                                                           std::string name)
  : name_(std::move(name))
  , longest_valid_segment_length_(longest_valid_segment_length)
  , contact_distance_(contact_distance)
{
}

const std::string& DiscreteContactCheckProcessGenerator::getName() const { return name_; }

std::function<void()> DiscreteContactCheckProcessGenerator::generateTask(ProcessInput input)
{
  return [=]() { process(input); };
}

std::function<int()> DiscreteContactCheckProcessGenerator::generateConditionalTask(ProcessInput input)
{
  return [=]() { return conditionalProcess(input); };
}

int DiscreteContactCheckProcessGenerator::conditionalProcess(ProcessInput input) const
{
  if (abort_)
    return 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  if (!isCompositeInstruction(*(input.results)))
  {
    CONSOLE_BRIDGE_logError("Input seed to TrajOpt Planner must be a composite instruction");
    return 0;
  }

  // Get state solver
  tesseract_environment::StateSolver::Ptr state_solver = input.tesseract->getEnvironmentConst()->getStateSolver();
  tesseract_collision::DiscreteContactManager::Ptr manager =
      input.tesseract->getEnvironmentConst()->getDiscreteContactManager();
  manager->setContactDistanceThreshold(contact_distance_);

  const auto* ci = input.results->cast_const<CompositeInstruction>();
  std::vector<tesseract_collision::ContactResultMap> contacts;
  if (contactCheckProgram(contacts, *manager, *state_solver, *ci, longest_valid_segment_length_))
  {
    CONSOLE_BRIDGE_logInform("Results are not contact free!");
    return 0;
  }

  CONSOLE_BRIDGE_logDebug("Discrete contact check succeeded");
  return 1;
}

void DiscreteContactCheckProcessGenerator::process(ProcessInput input) const { conditionalProcess(input); }

bool DiscreteContactCheckProcessGenerator::getAbort() const { return abort_; }
void DiscreteContactCheckProcessGenerator::setAbort(bool abort) { abort_ = abort; }

}  // namespace tesseract_planning
