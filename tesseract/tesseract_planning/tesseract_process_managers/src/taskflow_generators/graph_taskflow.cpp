/**
 * @file graph_taskflow.cpp
 * @brief Creates a directed graph taskflow
 *
 * @author Levi Armstrong
 * @date August 13. 2020
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
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/taskflow_generators/graph_taskflow.h>

namespace tesseract_planning
{
GraphTaskflow::GraphTaskflow(std::string name) : name_(std::move(name)) {}

const std::string& GraphTaskflow::getName() const { return name_; }

TaskflowContainer GraphTaskflow::generateTaskflow(ProcessInput input, TaskflowVoidFn done_cb, TaskflowVoidFn error_cb)
{
  // Create Taskflow and Container
  TaskflowContainer container;
  container.taskflow = std::make_unique<tf::Taskflow>(name_);

  // Add "Error" task
  if (error_cb)
    container.outputs.push_back(container.taskflow->emplace(error_cb).name("Error Callback"));
  else
    container.outputs.push_back(
        container.taskflow->emplace([&]() { std::cout << "Error GraphTaskflow\n"; }).name("Error Callback"));

  // Add "Done" task
  if (done_cb)
    container.outputs.push_back(container.taskflow->emplace(done_cb).name("Done Callback"));
  else
    container.outputs.push_back(
        container.taskflow->emplace([&]() { std::cout << "Done GraphTaskflow\n"; }).name("Done Callback"));

  // Generate process tasks for each node using its process generator
  std::vector<tf::Task> tasks;
  tasks.reserve(nodes_.size());
  for (auto& node : nodes_)
  {
    switch (node.process_type)
    {
      case NodeType::TASK:
      {
        tf::Task task = container.taskflow->placeholder();
        task.work(node.process->generateTask(input, task.hash_value()));
        task.name(node.process->getName());
        tasks.push_back(task);
        break;
      }
      case NodeType::CONDITIONAL:
      {
        tf::Task task = container.taskflow->placeholder();
        task.work(node.process->generateConditionalTask(input, task.hash_value()));
        task.name(node.process->getName());
        tasks.push_back(task);
        break;
      }
    }
  }

  std::size_t src_idx = 0;
  for (auto& node : nodes_)
  {
    std::size_t src = src_idx;
    if (node.process_type == NodeType::TASK)
    {
      assert(node.edges.size() == 1);
      if (node.edges[0].dest_channel == DestinationChannel::PROCESS_NODE)
        tasks[src].precede(tasks[static_cast<std::size_t>(node.edges[0].dest)]);
      else if (node.edges[0].dest_channel == DestinationChannel::DONE_CALLBACK)
        tasks[src].precede(container.outputs[1]);
      else if (node.edges[0].dest_channel == DestinationChannel::ERROR_CALLBACK)
        tasks[src].precede(container.outputs[0]);
    }
    else if (node.process_type == NodeType::CONDITIONAL)
    {
      assert(node.edges.size() == 2);
      if (node.edges[0].dest_channel == DestinationChannel::PROCESS_NODE &&
          node.edges[1].dest_channel == DestinationChannel::PROCESS_NODE)
      {
        if (node.edges[0].src_channel == SourceChannel::ON_SUCCESS &&
            node.edges[1].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(tasks[static_cast<std::size_t>(node.edges[1].dest)],
                             tasks[static_cast<std::size_t>(node.edges[0].dest)]);
        else if (node.edges[1].src_channel == SourceChannel::ON_SUCCESS &&
                 node.edges[0].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(tasks[static_cast<std::size_t>(node.edges[0].dest)],
                             tasks[static_cast<std::size_t>(node.edges[1].dest)]);
      }
      else if (node.edges[0].dest_channel == DestinationChannel::PROCESS_NODE &&
               node.edges[1].dest_channel == DestinationChannel::DONE_CALLBACK)
      {
        if (node.edges[0].src_channel == SourceChannel::ON_SUCCESS &&
            node.edges[1].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(container.outputs[1], tasks[static_cast<std::size_t>(node.edges[0].dest)]);
        else if (node.edges[1].src_channel == SourceChannel::ON_SUCCESS &&
                 node.edges[0].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(tasks[static_cast<std::size_t>(node.edges[0].dest)], container.outputs[1]);
      }
      else if (node.edges[0].dest_channel == DestinationChannel::PROCESS_NODE &&
               node.edges[1].dest_channel == DestinationChannel::ERROR_CALLBACK)
      {
        if (node.edges[0].src_channel == SourceChannel::ON_SUCCESS &&
            node.edges[1].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(container.outputs[0], tasks[static_cast<std::size_t>(node.edges[0].dest)]);
        else if (node.edges[1].src_channel == SourceChannel::ON_SUCCESS &&
                 node.edges[0].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(tasks[static_cast<std::size_t>(node.edges[0].dest)], container.outputs[0]);
      }
      else if (node.edges[0].dest_channel == DestinationChannel::DONE_CALLBACK &&
               node.edges[1].dest_channel == DestinationChannel::PROCESS_NODE)
      {
        if (node.edges[0].src_channel == SourceChannel::ON_SUCCESS &&
            node.edges[1].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(tasks[static_cast<std::size_t>(node.edges[1].dest)], container.outputs[1]);
        else if (node.edges[1].src_channel == SourceChannel::ON_SUCCESS &&
                 node.edges[0].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(container.outputs[1], tasks[static_cast<std::size_t>(node.edges[1].dest)]);
      }
      else if (node.edges[0].dest_channel == DestinationChannel::ERROR_CALLBACK &&
               node.edges[1].dest_channel == DestinationChannel::PROCESS_NODE)
      {
        if (node.edges[0].src_channel == SourceChannel::ON_SUCCESS &&
            node.edges[1].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(tasks[static_cast<std::size_t>(node.edges[1].dest)], container.outputs[0]);
        else if (node.edges[1].src_channel == SourceChannel::ON_SUCCESS &&
                 node.edges[0].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(container.outputs[0], tasks[static_cast<std::size_t>(node.edges[1].dest)]);
      }
      else if (node.edges[0].dest_channel == DestinationChannel::DONE_CALLBACK &&
               node.edges[1].dest_channel == DestinationChannel::ERROR_CALLBACK)
      {
        if (node.edges[0].src_channel == SourceChannel::ON_SUCCESS &&
            node.edges[1].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(container.outputs[0], container.outputs[1]);
        else if (node.edges[1].src_channel == SourceChannel::ON_SUCCESS &&
                 node.edges[0].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(container.outputs[1], container.outputs[0]);
      }
      else if (node.edges[0].dest_channel == DestinationChannel::ERROR_CALLBACK &&
               node.edges[1].dest_channel == DestinationChannel::DONE_CALLBACK)
      {
        if (node.edges[0].src_channel == SourceChannel::ON_SUCCESS &&
            node.edges[1].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(container.outputs[1], container.outputs[0]);
        else if (node.edges[1].src_channel == SourceChannel::ON_SUCCESS &&
                 node.edges[0].src_channel == SourceChannel::ON_FAILURE)
          tasks[src].precede(container.outputs[0], container.outputs[1]);
      }
      else
      {
        throw std::runtime_error("Invalide Edges for process index: " + std::to_string(src_idx));
      }
    }
    ++src_idx;
  }

  // Assumes the first node added is the input node
  container.input = tasks[0];
  return container;
}

int GraphTaskflow::addNode(ProcessGenerator::UPtr process, NodeType process_type)
{
  Node pn;
  pn.process = std::move(process);
  pn.process_type = process_type;

  nodes_.push_back(std::move(pn));

  return static_cast<int>(nodes_.size()) - 1;
}

void GraphTaskflow::addEdge(int src, SourceChannel src_channel, int dest, DestinationChannel dest_channel)
{
  Edge e;
  e.src_channel = src_channel;
  e.dest = dest;
  e.dest_channel = dest_channel;

  Node& n = nodes_[static_cast<std::size_t>(src)];
  n.edges.push_back(e);
  if (n.edges.size() > 2)
  {
    CONSOLE_BRIDGE_logWarn("Currently a node should not have more than two edges!");
  }
}
}  // namespace tesseract_planning
