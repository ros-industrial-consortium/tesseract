/**
 * @file waypoint_definitions.h
 * @brief The class that defines common types of waypoints that can be sent to the planners
 *
 * @author Matthew Powelson
 * @date February 29, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_PLANNERS_WAYPOINT_DEFINITIONS_H
#define TESSERACT_PLANNERS_WAYPOINT_DEFINITIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Dense>
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_motion_planners
{
/** @brief Used to specify the type of waypoint. Corresponds to a derived class of Waypoint*/
enum class WaypointType
{
  JOINT_WAYPOINT,
  JOINT_TOLERANCED_WAYPOINT,
  CARTESIAN_WAYPOINT,
};
/** @brief Defines a generic way of sending waypoints to a Tesseract Planner */
class Waypoint
{
public:
  using Ptr = std::shared_ptr<Waypoint>;
  using ConstPtr = std::shared_ptr<const Waypoint>;

  Waypoint() {}
  /** @brief Returns the type of waypoint so that it may be cast back to the derived type */
  WaypointType getType() const { return waypoint_type_; }
  /** @brief Used to weight different terms in the waypoint. (Optional)
   *
   * For example: joint 1 vs joint 2 of the same waypoint or waypoint 1 vs waypoint 2
   * Note: Each planner should define defaults for this when they are not set.*/
  Eigen::VectorXd coeffs_;
  /** @brief If false, this value is used as a guide rather than a rigid waypoint (Default=true)

  Example: In Trajopt, is_critical=true => constraint, is_critical=false => cost*/
  bool is_critical_ = true;

protected:
  /** @brief Should be set by the derived class for casting Waypoint back to appropriate derived class type */
  WaypointType waypoint_type_;
};
/** @brief Defines a joint position waypoint for use with Tesseract Planners*/
class JointWaypoint : public Waypoint
{
public:
  using Ptr = std::shared_ptr<JointWaypoint>;
  using ConstPtr = std::shared_ptr<const JointWaypoint>;

  // TODO: constructor that takes joint position vector
  JointWaypoint() { waypoint_type_ = WaypointType::JOINT_WAYPOINT; }
  /** Stores the joint values associated with this waypoint (radians). Must be in the same order as the joints in the
   * kinematics object*/
  Eigen::VectorXd joint_positions_;
};
/** @brief Defines a cartesian position waypoint for use with Tesseract Planners */
class CartesianWaypoint : public Waypoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CartesianWaypoint>;
  using ConstPtr = std::shared_ptr<const CartesianWaypoint>;

  CartesianWaypoint() { waypoint_type_ = WaypointType::CARTESIAN_WAYPOINT; }
  /** @brief Contains the position and orientation of this waypoint */
  Eigen::Isometry3d cartesian_position_;

  /** @brief Convenience function that returns the xyz cartesian position contained in cartesian_position_ */
  Eigen::Vector3d getPosition() { return cartesian_position_.translation(); }
  /**
   * @brief Convenience function that returns the wxyz rotation quarternion contained in cartesian_position
   * @return Quaternion(w, x, y, z)
   */
  Eigen::Vector4d getOrientation()
  {
    Eigen::Quaterniond q(cartesian_position_.rotation());
    return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
  }
};

/** @brief Defines a joint toleranced position waypoint for use with Tesseract Planners*/
class JointTolerancedWaypoint : public Waypoint
{
public:
  using Ptr = std::shared_ptr<JointTolerancedWaypoint>;
  using ConstPtr = std::shared_ptr<const JointTolerancedWaypoint>;

  // TODO: constructor that takes joint position vector
  JointTolerancedWaypoint() { waypoint_type_ = WaypointType::JOINT_TOLERANCED_WAYPOINT; }
  /** @brief Stores the joint values associated with this waypoint (radians) */
  Eigen::VectorXd joint_positions_;
  /** @brief Amount over joint_positions_ that is allowed (positive radians).

  The allowed range is joint_positions-lower_tolerance_ to joint_positions_+upper_tolerance*/
  Eigen::VectorXd upper_tolerance_;
  /** @brief Amount under joint_positions_ that is allowed (negative radians).

  The allowed range is joint_positions-lower_tolerance_ to joint_positions_+upper_tolerance*/
  Eigen::VectorXd lower_tolerance_;
};

}  // namespace tesseract_motion_planners
#endif
