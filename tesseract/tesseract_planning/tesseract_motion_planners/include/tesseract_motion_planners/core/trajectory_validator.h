/**
 * @file trajectory_validator.h
 * @brief Trajectory Validator Class.
 *
 * @author Michael Ripperger
 * @date March 16, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_CORE_TRAJECTORY_VALIDATOR_H
#define TESSERACT_MOTION_PLANNERS_CORE_TRAJECTORY_VALIDATOR_H

#include <type_traits>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_environment/core/utils.h>
#include <console_bridge/console.h>
#include <trajopt/utils.hpp>

namespace tesseract_motion_planners
{
/** @brief Enumeration of the types of validation checks that can be performed on a planned trajectory */
enum class PostPlanCheckType : unsigned
{
  NONE = 0x1,
  SINGLE_TIMESTEP_COLLISION = 0x2,
  DISCRETE_CONTINUOUS_COLLISION = 0x4,
  CAST_CONTINUOUS_COLLISION = 0x8
};

inline PostPlanCheckType operator|(PostPlanCheckType lhs, PostPlanCheckType rhs)
{
  using T = std::underlying_type<PostPlanCheckType>::type;
  return static_cast<PostPlanCheckType>(static_cast<T>(lhs) | static_cast<T>(rhs));
}

inline PostPlanCheckType operator&(PostPlanCheckType lhs, PostPlanCheckType rhs)
{
  using T = std::underlying_type<PostPlanCheckType>::type;
  return static_cast<PostPlanCheckType>(static_cast<T>(lhs) & static_cast<T>(rhs));
}

/**
 * @brief Class for performing validation checks on a planned trajectory
 */
class TrajectoryValidator
{
public:
  using Ptr = std::shared_ptr<TrajectoryValidator>;

  TrajectoryValidator(tesseract_collision::ContinuousContactManager::Ptr continuous_manager = nullptr,
                      tesseract_collision::DiscreteContactManager::Ptr discrete_manager = nullptr,
                      double longest_valid_segment_length = 0.01,
                      bool verbose = false)
    : continuous_contact_manager_(std::move(continuous_manager))
    , discrete_contact_manager_(std::move(discrete_manager))
    , longest_valid_segment_length_(longest_valid_segment_length)
    , verbose_(verbose)
  {
  }

  /**
   * @brief Performs checks on a planned trajectory to determine its validity
   * @param trajectory - the planned trajectory
   * @param check_type - the type of validation check to be performed
   * @param state_solver
   * @param joint_names
   * @return True if the trajectory is valid according per the check type, false otherwise
   */
  virtual bool trajectoryValid(const tesseract_common::TrajArray& trajectory,
                               const PostPlanCheckType& check_type,
                               const tesseract_environment::StateSolver& state_solver,
                               const std::vector<std::string>& joint_names,
                               const trajopt::SafetyMarginData::Ptr collision_pairs = nullptr)
  {
    bool valid = true;
    using T = std::underlying_type<PostPlanCheckType>::type;
    if (static_cast<T>(check_type & PostPlanCheckType::NONE) > 0)
    {
      CONSOLE_BRIDGE_logWarn("No post-plan trajectory validator specified; this is not advised");
    }
    tesseract_collision::ContactTestType type = tesseract_collision::ContactTestType::FIRST;
    bool requires_post_check = false;
    if (collision_pairs && collision_pairs->getNegativeCollisions())
    {
      requires_post_check = true;
      type = tesseract_collision::ContactTestType::ALL;
    }
    std::vector<tesseract_collision::ContactResultMap> contacts;

    // Check discrete collision at only the defined trajectory waypoints
    if (static_cast<T>(check_type & PostPlanCheckType::SINGLE_TIMESTEP_COLLISION) > 0)
    {
      CONSOLE_BRIDGE_logInform("Performing discrete, single timestep collision check");
      if (discrete_contact_manager_)
      {
        bool in_collision = tesseract_environment::checkTrajectory(contacts,
                                                                   *discrete_contact_manager_,
                                                                   state_solver,
                                                                   joint_names,
                                                                   trajectory,
                                                                   type,
                                                                   verbose_);
        valid &= !in_collision;
      }
      else
      {
        CONSOLE_BRIDGE_logError("Discrete contact manager not initialized!");
        valid = false;
      }
    }

    // Check discrete collision at waypoints generated by joint-interpolating the input trajectory at a specified
    // resolution
    if (static_cast<T>(check_type & PostPlanCheckType::DISCRETE_CONTINUOUS_COLLISION) > 0)
    {
      CONSOLE_BRIDGE_logInform("Performing discrete continuous collision check");
      if (discrete_contact_manager_)
      {
        bool in_collision = tesseract_environment::checkTrajectory(contacts,
                                                                   *discrete_contact_manager_,
                                                                   state_solver,
                                                                   joint_names,
                                                                   trajectory,
                                                                   longest_valid_segment_length_,
                                                                   type,
                                                                   verbose_);
        valid &= !in_collision;
      }
      else
      {
        CONSOLE_BRIDGE_logError("Discrete contact manager not initialized!");
        valid = false;
      }
    }

    // Check continuous collision between waypoints generated by joint-interpolating the input trajectory at a specified
    // resolution
    if (static_cast<T>(check_type & PostPlanCheckType::CAST_CONTINUOUS_COLLISION) > 0)
    {
      CONSOLE_BRIDGE_logInform("Performing cast continuous collision check");
      if (continuous_contact_manager_)
      {
        bool in_collision = tesseract_environment::checkTrajectory(contacts,
                                                                   *continuous_contact_manager_,
                                                                   state_solver,
                                                                   joint_names,
                                                                   trajectory,
                                                                   longest_valid_segment_length_,
                                                                   type,
                                                                   verbose_);
        valid &= !in_collision;
      }
      else
      {
        CONSOLE_BRIDGE_logError("Continuous contact manager not initialized!");
        valid = false;
      }
    }

    if (!valid && requires_post_check)
    {
      valid = false;
      for (auto contact_state : contacts)
      {
        for (auto collision : contact_state)
        {
          std::string first_link, second_link;
          first_link = collision.first.first;
          second_link = collision.first.second;
          Eigen::Vector2d coll_info = collision_pairs->getPairSafetyMarginData(first_link, second_link);
          if (collision.second.front().distance <= coll_info.x())
          {
            CONSOLE_BRIDGE_logInform("Found unallowed collision between %s and %s at a distance of %f",
                                     first_link.c_str(),
                                     second_link.c_str(),
                                     collision.second.front().distance);
            valid = false;
          }
          else
          {
            CONSOLE_BRIDGE_logInform("Found allowed collision between %s and %s at a distance of %f",
                                     first_link.c_str(),
                                     second_link.c_str(),
                                     collision.second.front().distance);
            valid = true;
          }
        }
        if (!valid)
          break;
      }
    }

    return valid;
  }

protected:
  tesseract_collision::ContinuousContactManager::Ptr continuous_contact_manager_;
  tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager_;

  double longest_valid_segment_length_;
  bool verbose_;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_CORE_TRAJECTORY_VALIDATOR_H
