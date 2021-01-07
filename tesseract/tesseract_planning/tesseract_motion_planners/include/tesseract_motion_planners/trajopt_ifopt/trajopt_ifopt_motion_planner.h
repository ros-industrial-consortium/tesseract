/**
 * @file trajopt_planner.h
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_IFOPT_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ifopt/problem.h>
#include <trajopt_sqp/sqp_callback.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>

namespace tesseract_planning
{
class TrajOptIfoptMotionPlannerStatusCategory;

using TrajOptIfoptProblemGeneratorFn =
    std::function<std::shared_ptr<TrajOptIfoptProblem>(const std::string&,
                                                       const PlannerRequest&,
                                                       const TrajOptIfoptPlanProfileMap&,
                                                       const TrajOptIfoptCompositeProfileMap&)>;

class TrajOptIfoptMotionPlanner : public MotionPlanner
{
public:
  /** @brief Construct a basic planner */
  TrajOptIfoptMotionPlanner();

  ~TrajOptIfoptMotionPlanner() override = default;
  TrajOptIfoptMotionPlanner(const TrajOptIfoptMotionPlanner&) = delete;
  TrajOptIfoptMotionPlanner& operator=(const TrajOptIfoptMotionPlanner&) = delete;
  TrajOptIfoptMotionPlanner(TrajOptIfoptMotionPlanner&&) = delete;
  TrajOptIfoptMotionPlanner& operator=(TrajOptIfoptMotionPlanner&&) = delete;

  const std::string& getName() const override;

  TrajOptIfoptProblemGeneratorFn problem_generator;

  /**
   * @brief The available composite profiles
   *
   * Composite instruction is a way to namespace or organize your planning problem. The composite instruction has a
   * profile which is used for applying multy waypoint costs and constraints like joint smoothing, collision avoidance,
   * and velocity smoothing.
   */
  TrajOptIfoptCompositeProfileMap composite_profiles;

  /**
   * @brief The available plan profiles
   *
   * Plan instruction profiles are used to control waypoint specific information like fixed waypoint, toleranced
   * waypoint, corner distance waypoint, etc.
   */
  TrajOptIfoptPlanProfileMap plan_profiles;

  /** @brief Optimization parameters to be used (Optional) */
  //  sco::BasicTrustRegionSQPParameters params;

  /** @brief Callback functions called on each iteration of the optimization (Optional) */
  std::vector<trajopt_sqp::SQPCallback::Ptr> callbacks;

  /**
   * @brief Sets up the opimizer and solves a SQP problem read from json with no callbacks and dafault parameterss
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @param check_type The type of collision check to perform on the planned trajectory
   * @param verbose Boolean indicating whether logging information about the motion planning solution should be printed
   * to console
   * @return true if optimization complete
   */
  tesseract_common::StatusCode solve(const PlannerRequest& request,
                                     PlannerResponse& response,
                                     bool verbose = false) const override;

  bool checkUserInput(const PlannerRequest& request) const;

  bool terminate() override;

  void clear() override;

protected:
  /** @brief Name of planner */
  std::string name_{ "TrajOptIfopt" };

  std::shared_ptr<const TrajOptIfoptMotionPlannerStatusCategory> status_category_; /** @brief The planners status codes
                                                                                    */
};

class TrajOptIfoptMotionPlannerStatusCategory : public tesseract_common::StatusCategory
{
public:
  TrajOptIfoptMotionPlannerStatusCategory(std::string name);
  const std::string& name() const noexcept override;
  std::string message(int code) const override;

  enum
  {
    SolutionFound = 0,
    ErrorInvalidInput = -1,
    FailedToFindValidSolution = -3,
  };

private:
  std::string name_;
};

}  // namespace tesseract_planning
#endif  // TESSERACT_PLANNING_TRAJOPT_IFOPT_PLANNER_H
