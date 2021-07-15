/**
 * @file ikfast_inv_kin.h
 * @brief Tesseract IKFast Inverse kinematics Wrapper
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_KINEMATICS_IKFAST_INV_KIN_H
#define TESSERACT_KINEMATICS_IKFAST_INV_KIN_H

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/types.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::IKFastInvKin)
#endif  // SWIG

namespace tesseract_kinematics
{
/**
 * @brief IKFast Inverse Kinematics Implmentation.
 *
 * This along with the ikfast_inv_kin.hpp is to be used with a generated ikfast to create a tesseract implementation.
 * Once you have created your ikfast solver of your robot all you need is to create header similiar to what is shown
 * below:
 *
 * Header File: fanuc_p50ib_15_inv_kinematics.h
 *
 * #include <Eigen/Geometry>
 * #include <vector>
 * #include <tesseract_kinematics/ikfast/ikfast_inv_kin.h>
 * #include <tesseract_kinematics/core/forward_kinematics.h>

 * namespace fanuc_p50ib_15_ikfast_wrapper
 * {
 * class FanucP50iBInvKinematics : public tesseract_kinematics::IKFastInvKin
 * {
 * public:
 *   FanucP50iBInvKinematics(const std::string name,
 *                           const std::string base_link_name,
 *                           const std::string tip_link_name,
 *                           const std::vector<std::string> joint_names,
 *                           const std::vector<std::string> link_names,
 *                           const std::vector<std::string> active_link_names,
 *                           const Eigen::MatrixX2d joint_limits)
 * };
 *
 * Cpp File:
 *
 * #include <tesseract_kinematics/ikfast/impl/ikfast_inv_kin.hpp>
 * #include <fanuc_p50ib_15_ikfast_wrapper/impl/fanuc_p50ib_15_ikfast.hpp>
 * #include <fanuc_p50ib_15_ikfast_wrapper/tesseract_fanuc_p50ib_kinematics.h>
 *
 * namespace fanuc_p50ib_15_ikfast_wrapper
 * {
 *   FanucP50iBInvKinematics::FanucP50iBInvKinematics(const std::string name,
 *                                                    const std::string base_link_name,
 *                                                    const std::string tip_link_name,
 *                                                    const std::vector<std::string> joint_names,
 *                                                    const std::vector<std::string> link_names,
 *                                                    const std::vector<std::string> active_link_names,
 *                                                    const Eigen::MatrixX2d& joint_limits)
 *   : FanucP50iBInvKinematics(name, base_link_name, tip_link_name, joint_names, link_names, active_link_names,
 joint_limits)
 *   {}
 * }
 *
*/
class IKFastInvKin : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<IKFastInvKin>;
  using ConstPtr = std::shared_ptr<const IKFastInvKin>;

  IKFastInvKin() = default;
  ~IKFastInvKin() override = default;
  IKFastInvKin(const IKFastInvKin&) = delete;
  IKFastInvKin& operator=(const IKFastInvKin&) = delete;
  IKFastInvKin(IKFastInvKin&&) = delete;
  IKFastInvKin& operator=(IKFastInvKin&&) = delete;

  InverseKinematics::Ptr clone() const override;

  bool update() override;

  void synchronize(ForwardKinematics::ConstPtr fwd_kin) override;
  bool isSynchronized() const override;

  IKSolutions calcInvKin(const Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& seed) const override;

  IKSolutions calcInvKin(const Eigen::Isometry3d& pose,
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
   * @brief Initialize IKFast Inverse Kinematics
   * @param name The name of the kinematic chain
   * @param base_link_name The name of the base link for the kinematic chain
   * @param tip_link_name The name of the tip link for the kinematic chain
   * @param joint_names The joint names for the kinematic chain
   * @param link_names The link names for the kinematic chain
   * @param active_link_names The active links names for the kinematic chain
   * @param joint_limits The joint limits for the kinematic chain
   * @return True if successful
   */
  bool init(std::string name,
            std::string base_link_name,
            std::string tip_link_name,
            std::vector<std::string> joint_names,
            std::vector<std::string> link_names,
            std::vector<std::string> active_link_names,
            tesseract_common::KinematicLimits limits,
            std::vector<Eigen::Index> redundancy_indices);

  /**
   * @brief Checks if kinematics has been initialized
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const;

protected:
  bool initialized_ = false;                  /**< @brief Identifies if the object has been initialized */
  ForwardKinematics::ConstPtr sync_fwd_kin_;  /**< @brief Synchronized forward kinematics object */
  std::vector<Eigen::Index> sync_joint_map_;  /**< @brief Synchronized joint solution remapping */
  SynchronizableData data_;                   /**< @brief The current data that may be synchronized */
  SynchronizableData orig_data_;              /**< @brief The data prior to synchronization */
  std::string name_;                          /**< @brief Name of the kinematic chain */
  std::string solver_name_{ "IKFastInvKin" }; /**< @brief Name of this solver */

  /**
   * @brief This used by the clone method
   * @return True if init() completes successfully
   */
  bool init(const IKFastInvKin& kin);
};

}  // namespace tesseract_kinematics

#include <tesseract_kinematics/ikfast/impl/ikfast_inv_kin.hpp>

#endif  // TESSERACT_KINEMATICS_IKFAST_INV_KIN_H
