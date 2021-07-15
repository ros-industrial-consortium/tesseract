/**
 * @file kdl_inv_kin_chain_lma.h
 * @brief Tesseract KDL Inverse kinematics chain Levenberg-Marquardt implementation.
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
#ifndef TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_LMA_H
#define TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_LMA_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <unordered_map>
#include <console_bridge/console.h>

#include <tesseract_scene_graph/graph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/types.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::KDLInvKinChainLMA)
#endif  // SWIG

namespace tesseract_kinematics
{
/**
 * @brief KDL Inverse kinematic chain implementation.
 */
class KDLInvKinChainLMA : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<KDLInvKinChainLMA>;
  using ConstPtr = std::shared_ptr<const KDLInvKinChainLMA>;

  KDLInvKinChainLMA() = default;
  ~KDLInvKinChainLMA() override = default;
  KDLInvKinChainLMA(const KDLInvKinChainLMA&) = delete;
  KDLInvKinChainLMA& operator=(const KDLInvKinChainLMA&) = delete;
  KDLInvKinChainLMA(KDLInvKinChainLMA&&) = delete;
  KDLInvKinChainLMA& operator=(KDLInvKinChainLMA&&) = delete;

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

  unsigned int numJoints() const override;

  const std::string& getBaseLinkName() const override;

  const std::string& getTipLinkName() const override;

  const std::string& getName() const override;

  const std::string& getSolverName() const override;

  tesseract_scene_graph::SceneGraph::ConstPtr getSceneGraph() const;

  /**
   * @brief Initializes Inverse Kinematics as chain
   * Creates a inverse kinematic chain object
   * @param scene_graph The Tesseract Scene Graph
   * @param base_link The name of the base link for the kinematic chain
   * @param tip_link The name of the tip link for the kinematic chain
   * @param name The name of the kinematic chain
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
            const std::string& base_link,
            const std::string& tip_link,
            std::string name);

  /**
   * @brief Initializes Inverse Kinematics as chain
   * Creates a inverse kinematic chain object from sequential chains
   * @param scene_graph The Tesseract Scene Graph
   * @param chains A vector of kinematics chains <base_link, tip_link> that get concatenated
   * @param name The name of the kinematic chain
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
            const std::vector<std::pair<std::string, std::string> >& chains,
            std::string name);

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
  KDLChainData kdl_data_;                                   /**< @brief KDL data parsed from Scene Graph */
  SynchronizableData orig_data_;                            /**< @brief The data prior to synchronization */
  std::string name_;                                        /**< @brief Name of the kinematic chain */
  std::string solver_name_{ "KDLInvKinChainLMA" };          /**< @brief Name of this solver */
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;    /**< @brief KDL Inverse kinematic solver */

  /**
   * @brief This used by the clone method
   * @return True if init() completes successfully
   */
  bool init(const KDLInvKinChainLMA& kin);

  /** @brief calcFwdKin helper function */
  IKSolutions calcInvKinHelper(const Eigen::Isometry3d& pose,
                               const Eigen::Ref<const Eigen::VectorXd>& seed,
                               int segment_num = -1) const;
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_LMA_H
