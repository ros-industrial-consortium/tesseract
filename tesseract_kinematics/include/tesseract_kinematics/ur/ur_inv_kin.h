/*********************************************************************
 *
 * Provides forward and inverse kinematics for Univeral robot designs
 * Author: Kelsey Hawkins (kphawkins@gatech.edu)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Georgia Institute of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef TESSERACT_KINEMATICS_UR_INV_KIN_H
#define TESSERACT_KINEMATICS_UR_INV_KIN_H

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/types.h>

namespace tesseract_kinematics
{
/**@brief Universal Robot Inverse Kinematics Implmentation. */
class URInvKin : public tesseract_kinematics::InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<URInvKin>;
  using ConstPtr = std::shared_ptr<const URInvKin>;

  URInvKin() = default;
  ~URInvKin() override = default;
  URInvKin(const URInvKin&) = delete;
  URInvKin& operator=(const URInvKin&) = delete;
  URInvKin(URInvKin&&) = delete;
  URInvKin& operator=(URInvKin&&) = delete;

  tesseract_kinematics::InverseKinematics::Ptr clone() const override;

  bool update() override;

  void synchronize(ForwardKinematics::ConstPtr fwd_kin) override;
  bool isSynchronized() const override;

  tesseract_kinematics::IKSolutions calcInvKin(const Eigen::Isometry3d& pose,
                                               const Eigen::Ref<const Eigen::VectorXd>& seed) const override;

  tesseract_kinematics::IKSolutions calcInvKin(const Eigen::Isometry3d& pose,
                                               const Eigen::Ref<const Eigen::VectorXd>& seed,
                                               const std::string& link_name) const override;

  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const override;
  unsigned int numJoints() const override;

  const std::vector<std::string>& getJointNames() const override;
  const std::vector<std::string>& getLinkNames() const override;
  const std::vector<std::string>& getActiveLinkNames() const override;
  const tesseract_common::KinematicLimits& getLimits() const override;
  void setLimits(tesseract_common::KinematicLimits limits) override;
  std::vector<Eigen::Index> getRedundancyCapableJointIndices() const override;
  const std::string& getBaseLinkName() const override;
  const std::string& getTipLinkName() const override;
  const std::string& getName() const override;
  const std::string& getSolverName() const override;

  /**
   * @brief init Initialize UR Inverse Kinematics
   * @param name The name of the kinematic chain
   * @param params UR kinematics parameters
   * @param base_link_name The name of the base link for the kinematic chain
   * @param tip_link_name The name of the tip link for the kinematic chain
   * @param joint_names The joint names for the kinematic chain
   * @param link_names The link names for the kinematic chain
   * @param active_link_names The active links names for the kinematic chain
   * @param joint_limits The joint limits for the kinematic chain
   * @return True if successful
   */
  bool init(std::string name,
            URParameters params,
            std::string base_link_name,
            std::string tip_link_name,
            std::vector<std::string> joint_names,
            std::vector<std::string> link_names,
            std::vector<std::string> active_link_names,
            tesseract_common::KinematicLimits limits);

  /**
   * @brief Checks if kinematics has been initialized
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const;

protected:
  bool initialized_{ false };                /**< @brief Identifies if the object has been initialized */
  URParameters params_;                      /**< @brief The UR Inverse kinematics parameters */
  ForwardKinematics::ConstPtr sync_fwd_kin_; /**< @brief Synchronized forward kinematics object */
  std::vector<Eigen::Index> sync_joint_map_; /**< @brief Synchronized joint solution remapping */
  SynchronizableData data_;                  /**< @brief The current data that may be synchronized */
  SynchronizableData orig_data_;             /**< @brief The data prior to synchronization */
  std::string name_;                         /**< @brief Name of the kinematic chain */
  std::string solver_name_{ "URInvKin" };    /**< @brief Name of this solver */

  /**
   * @brief This used by the clone method
   * @return True if init() completes successfully
   */
  bool init(const URInvKin& kin);
};
}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_UR_INV_KIN_H
