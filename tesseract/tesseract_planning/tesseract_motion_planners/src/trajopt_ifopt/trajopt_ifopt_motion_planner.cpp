/**
 * @file trajopt_planner.cpp
 * @brief Tesseract ROS Trajopt planner
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <tesseract_environment/core/utils.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_motion_planner.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/core/utils.h>

using namespace trajopt;

namespace tesseract_planning
{
TrajOptIfoptMotionPlannerStatusCategory::TrajOptIfoptMotionPlannerStatusCategory(std::string name)
  : name_(std::move(name))
{
}
const std::string& TrajOptIfoptMotionPlannerStatusCategory::name() const noexcept { return name_; }
std::string TrajOptIfoptMotionPlannerStatusCategory::message(int code) const
{
  switch (code)
  {
    case SolutionFound:
    {
      return "Found valid solution";
    }
    case ErrorInvalidInput:
    {
      return "Input to planner is invalid. Check that instructions and seed are compatible";
    }
    case FailedToFindValidSolution:
    {
      return "Failed to find valid solution";
    }
    default:
    {
      assert(false);
      return "";
    }
  }
}

TrajOptIfoptMotionPlanner::TrajOptIfoptMotionPlanner()
  : status_category_(std::make_shared<const TrajOptIfoptMotionPlannerStatusCategory>(name_))
{
}

const std::string& TrajOptIfoptMotionPlanner::getName() const { return name_; }

bool TrajOptIfoptMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

void TrajOptIfoptMotionPlanner::clear()
{
  //  params = sco::BasicTrustRegionSQPParameters();
  callbacks.clear();
}

tesseract_common::StatusCode TrajOptIfoptMotionPlanner::solve(const PlannerRequest& request,
                                                              PlannerResponse& response,
                                                              bool verbose) const
{
  if (!checkUserInput(request))
  {
    response.status =
        tesseract_common::StatusCode(TrajOptIfoptMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }
  std::shared_ptr<TrajOptIfoptProblem> problem;
  if (request.data)
  {
    problem = std::static_pointer_cast<TrajOptIfoptProblem>(request.data);
  }
  else
  {
    if (!problem_generator)
    {
      CONSOLE_BRIDGE_logError("TrajOptIfoptPlanner does not have a problem generator specified.");
      response.status =
          tesseract_common::StatusCode(TrajOptIfoptMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
      return response.status;
    }
    problem = problem_generator(name_, request, plan_profiles, composite_profiles);
    response.data = problem;
  }

  // Create optimizer
  /** @todo Enable solver selection (e.g. IPOPT) */
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  /** @todo Set these as the defaults in trajopt and allow setting */
  qp_solver->solver_.settings()->setVerbosity(verbose);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
  qp_solver->solver_.settings()->setMaxIteraction(8192);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  // Add all callbacks
  for (const trajopt_sqp::SQPCallback::Ptr& callback : callbacks)
  {
    solver.registerCallback(callback);
  }

  // solve
  solver.verbose = verbose;

  auto tStart = boost::posix_time::second_clock::local_time();
  solver.Solve(*(problem->nlp));
  CONSOLE_BRIDGE_logInform("planning time: %.3f", (boost::posix_time::second_clock::local_time() - tStart).seconds());

  // Check success
  if (solver.getStatus() != trajopt_sqp::SQPStatus::NLP_CONVERGED)
  {
    response.status = tesseract_common::StatusCode(TrajOptIfoptMotionPlannerStatusCategory::FailedToFindValidSolution,
                                                   status_category_);
    return response.status;
  }

  // Get the results - This can likely be simplified if we get rid of the traj array
  Eigen::VectorXd x = problem->nlp->GetOptVariables()->GetValues();
  Eigen::Map<trajopt::TrajArray> trajectory(x.data(),
                                            static_cast<Eigen::Index>(problem->vars.size()),
                                            static_cast<Eigen::Index>(problem->vars[0]->GetValues().size()));

  // Flatten the results to make them easier to process
  response.results = request.seed;
  auto results_flattened = flattenProgramToPattern(response.results, request.instructions);
  auto instructions_flattened = flattenProgram(request.instructions);

  // Loop over the flattened results and add them to response if the input was a plan instruction
  Eigen::Index result_index = 0;
  for (std::size_t idx = 0; idx < instructions_flattened.size(); idx++)
  {
    // If idx is zero then this should be the start instruction
    assert((idx == 0) ? isPlanInstruction(instructions_flattened.at(idx).get()) : true);
    assert((idx == 0) ? isMoveInstruction(results_flattened[idx].get()) : true);
    if (isPlanInstruction(instructions_flattened.at(idx).get()))
    {
      // This instruction corresponds to a composite. Set all results in that composite to the results
      const auto* plan_instruction = instructions_flattened.at(idx).get().cast_const<PlanInstruction>();
      if (plan_instruction->isStart())
      {
        assert(idx == 0);
        assert(isMoveInstruction(results_flattened[idx].get()));
        auto* move_instruction = results_flattened[idx].get().cast<MoveInstruction>();
        move_instruction->getWaypoint().cast<StateWaypoint>()->position = trajectory.row(result_index++);
      }
      else
      {
        auto* move_instructions = results_flattened[idx].get().cast<CompositeInstruction>();
        for (auto& instruction : *move_instructions)
          instruction.cast<MoveInstruction>()->getWaypoint().cast<StateWaypoint>()->position =
              trajectory.row(result_index++);
      }
    }
  }

  response.status =
      tesseract_common::StatusCode(TrajOptIfoptMotionPlannerStatusCategory::SolutionFound, status_category_);
  return response.status;
}

bool TrajOptIfoptMotionPlanner::checkUserInput(const PlannerRequest& request) const
{
  // Check that parameters are valid
  if (request.env == nullptr)
  {
    CONSOLE_BRIDGE_logError("In TrajOptPlannerUniversalConfig: tesseract is a required parameter and has not been set");
    return false;
  }

  if (request.instructions.empty())
  {
    CONSOLE_BRIDGE_logError("TrajOptPlannerUniversalConfig requires at least one instruction");
    return false;
  }

  return true;
}

}  // namespace tesseract_planning
