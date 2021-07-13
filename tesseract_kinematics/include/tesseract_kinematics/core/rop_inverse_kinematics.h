/**
 * @file rop_inverse_kinematics.h
 * @brief Robot on Positioner Inverse kinematics functions.
 *
 * @author Levi Armstrong
 * @date June 25, 2020
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
#ifndef TESSERACT_KINEMATICS_ROP_INVERSE_KINEMATICS_H
#define TESSERACT_KINEMATICS_ROP_INVERSE_KINEMATICS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <unordered_map>
#include <console_bridge/console.h>

#include <tesseract_scene_graph/graph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/types.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::RobotOnPositionerInvKin)
#endif  // SWIG

namespace tesseract_kinematics
{
/**
 * @brief Robot on Positioner Inverse kinematic implementation.
 */
class RobotOnPositionerInvKin : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<RobotOnPositionerInvKin>;
  using ConstPtr = std::shared_ptr<const RobotOnPositionerInvKin>;

  RobotOnPositionerInvKin() = default;
  ~RobotOnPositionerInvKin() override = default;
  RobotOnPositionerInvKin(const RobotOnPositionerInvKin&) = delete;
  RobotOnPositionerInvKin& operator=(const RobotOnPositionerInvKin&) = delete;
  RobotOnPositionerInvKin(RobotOnPositionerInvKin&&) = delete;
  RobotOnPositionerInvKin& operator=(RobotOnPositionerInvKin&&) = delete;

  InverseKinematics::Ptr clone() const override;

  bool update() override;

  void synchronize(ForwardKinematics::ConstPtr fwd_kin) override;
  bool isSynchronized() const override;

  IKSolutions calcInvKin(const Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& seed) const override;

  IKSolutions calcInvKin(const Eigen::Isometry3d& pose,
                         const Eigen::Ref<const Eigen::VectorXd>& seed,
                         const std::string& link_name) const override;

  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const override;

  const std::vector<std::string>& getJointNames() const override;

  const std::vector<std::string>& getLinkNames() const override;

  const std::vector<std::string>& getActiveLinkNames() const override;

  const tesseract_common::KinematicLimits& getLimits() const override;

  void setLimits(tesseract_common::KinematicLimits limits) override;

  std::vector<Eigen::Index> getRedundancyCapableJointIndices() const override;

  tesseract_scene_graph::SceneGraph::ConstPtr getSceneGraph() const;
  unsigned int numJoints() const override;
  const std::string& getBaseLinkName() const override;
  const std::string& getTipLinkName() const override;
  const std::string& getName() const override;
  const std::string& getSolverName() const override;

  /**
   * @brief Initializes Inverse Kinematics for a robot on a positioner
   * @param scene_graph The Tesseract Scene Graph
   * @param manipulator
   * @param manipulator_reach
   * @param positioner
   * @param positioner_sample_resolution
   * @param name The name of the kinematic object
   * @param solver_name The name given to the solver. This is exposed so you may have same solver with different
   * sampling resolutions
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
            InverseKinematics::Ptr manipulator,
            double manipulator_reach,
            ForwardKinematics::Ptr positioner,
            Eigen::VectorXd positioner_sample_resolution,
            std::string name,
            std::string solver_name = "RobotOnPositionerInvKin");

  /**
   * @brief Checks if kinematics has been initialized
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const;

private:
  bool initialized_{ false };                               /**< @brief Identifies if the object has been initialized */
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_; /**< @brief Tesseract Scene Graph */
  ForwardKinematics::ConstPtr sync_fwd_kin_;                /**< @brief Synchronized forward kinematics object */
  std::vector<Eigen::Index> sync_joint_map_;                /**< @brief Synchronized joint solution remapping */
  InverseKinematics::Ptr manip_inv_kin_;
  double manip_reach_{ 0 };
  ForwardKinematics::Ptr positioner_fwd_kin_;
  Eigen::VectorXd positioner_sample_resolution_;
  unsigned dof_;
  SynchronizableData data_;      /**< @brief The current data that may be synchronized */
  SynchronizableData orig_data_; /**< @brief The data prior to synchronization */
  std::vector<Eigen::VectorXd> dof_range_;
  std::string name_;                                     /**< @brief Name of the kinematic chain */
  std::string solver_name_{ "RobotOnPositionerInvKin" }; /**< @brief Name of this solver */

  /**
   * @brief This used by the clone method
   * @return True if init() completes successfully
   */
  bool init(const RobotOnPositionerInvKin& kin);

  /** @brief calcFwdKin helper function */
  IKSolutions calcInvKinHelper(const Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  void nested_ik(IKSolutions& solutions,
                 int loop_level,
                 const std::vector<Eigen::VectorXd>& dof_range,
                 const Eigen::Isometry3d& target_pose,
                 Eigen::VectorXd& positioner_pose,
                 const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  void ikAt(IKSolutions& solutions,
            const Eigen::Isometry3d& target_pose,
            Eigen::VectorXd& positioner_pose,
            const Eigen::Ref<const Eigen::VectorXd>& seed) const;
};
}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_ROP_INVERSE_KINEMATICS_H
