﻿/**
 * @file motion_planner_process_generator.h
 * @brief Generates a motion planning process
 *
 * @author Matthew Powelson
 * @date July 15. 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_MOTION_PLANNER_PROCESS_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_MOTION_PLANNER_PROCESS_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <console_bridge/console.h>
#include <atomic>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_command_language/null_instruction.h>

#include <tesseract_process_managers/process_generator.h>

namespace tesseract_planning
{
class MotionPlannerProcessGenerator : public ProcessGenerator
{
public:
  using UPtr = std::unique_ptr<MotionPlannerProcessGenerator>;

  MotionPlannerProcessGenerator(std::shared_ptr<MotionPlanner> planner);
  ~MotionPlannerProcessGenerator() override = default;
  MotionPlannerProcessGenerator(const MotionPlannerProcessGenerator&) = delete;
  MotionPlannerProcessGenerator& operator=(const MotionPlannerProcessGenerator&) = delete;
  MotionPlannerProcessGenerator(MotionPlannerProcessGenerator&&) = delete;
  MotionPlannerProcessGenerator& operator=(MotionPlannerProcessGenerator&&) = delete;

  const std::string& getName() const override;

  std::function<void()> generateTask(ProcessInput input) override;

  std::function<int()> generateConditionalTask(ProcessInput input) override;

  bool getAbort() const override;

  void setAbort(bool abort) override;

private:
  /** @brief If true, all tasks return immediately. Workaround for https://github.com/taskflow/taskflow/issues/201 */
  std::atomic<bool> abort_{ false };

  std::shared_ptr<MotionPlanner> planner_{ nullptr };

  std::string name_;

  int conditionalProcess(ProcessInput input) const;

  void process(ProcessInput input) const;
};

}  // namespace tesseract_planning

#endif
