﻿/**
 * @file process_generator.h
 * @brief Process generator
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROCESS_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_PROCESS_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <memory>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/process_input.h>

namespace tesseract_planning
{
/**
 * @brief This is a base class for generating instances of processes as tasks such that they may be executed in
 * parallel. A typical workflow would be task t = process_generator.generateTask(input, taskflow)
 *
 * Only unique pointers should be used because of the ability to abort the process. With recent changes this may no
 * longer be valid but need to investigate.
 */
class ProcessGenerator
{
public:
  using UPtr = std::unique_ptr<ProcessGenerator>;

  ProcessGenerator(std::string name = "");
  virtual ~ProcessGenerator() = default;
  ProcessGenerator(const ProcessGenerator&) = delete;
  ProcessGenerator& operator=(const ProcessGenerator&) = delete;
  ProcessGenerator(ProcessGenerator&&) = delete;
  ProcessGenerator& operator=(ProcessGenerator&&) = delete;

  /**
   * @brief Get the task name
   * @return The name
   */
  virtual const std::string& getName() const;

  /**
   * @brief Generated a Task
   * @param input The process input
   * @param taskflow The taskflow to associate the task with
   * @return Task
   */
  virtual tf::Task generateTask(ProcessInput input, tf::Taskflow& taskflow);

  /**
   * @brief Assign work to the provided task
   * @param input The process input
   * @param task The task to assign the work to
   */
  virtual void assignTask(ProcessInput input, tf::Task& task);

  /**
   * @brief Generated a Task
   * @param input The process input
   * @param taskflow The taskflow to associate the task with
   * @return Conditional Task
   */
  virtual tf::Task generateConditionalTask(ProcessInput input, tf::Taskflow& taskflow);

  /**
   * @brief Assign work to the provided task
   * @param input The process input
   * @param task The task to assign the work to
   */
  virtual void assignConditionalTask(ProcessInput input, tf::Task& task);

protected:
  /** @brief The name of the process */
  std::string name_;

  /**
   * @brief Generated a Task
   * @param input The process input
   * @return Task
   */
  virtual void process(ProcessInput input, std::size_t unique_id) const = 0;

  /**
   * @brief Generate Conditional Task
   * @param input The process input
   * @return Conditional Task
   */
  virtual int conditionalProcess(ProcessInput input, std::size_t unique_id) const = 0;
};

}  // namespace tesseract_planning

#endif
