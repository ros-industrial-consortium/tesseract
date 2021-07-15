/**
 * @file kdl_kinematic_chain.cpp
 * @brief Tesseract KDL kinematics chain implementation.
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/segment.hpp>
#include <tesseract_scene_graph/parser/kdl_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

ForwardKinematics::Ptr KDLFwdKinChain::clone() const
{
  auto cloned_fwdkin = std::make_shared<KDLFwdKinChain>();
  cloned_fwdkin->init(*this);
  return cloned_fwdkin;
}

bool KDLFwdKinChain::update()
{
  return init(scene_graph_, kdl_data_.data.base_link_name, kdl_data_.data.tip_link_name, name_);
}

Eigen::Isometry3d KDLFwdKinChain::calcFwdKinHelper(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                                   int segment_num) const
{
  KDL::JntArray kdl_joints;
  EigenToKDL(joint_angles, kdl_joints);

  // run FK solver
  KDL::Frame kdl_pose;
  if (fk_solver_->JntToCart(kdl_joints, kdl_pose, segment_num) < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to calculate FK");
    throw std::runtime_error("KDLFwdKinChain: Failed to calculate forward kinematics.");
  }

  Eigen::Isometry3d pose;
  KDLToEigen(kdl_pose, pose);

  return pose;
}

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH

tesseract_common::VectorIsometry3d
KDLFwdKinChain::calcFwdKinHelperAll(const Eigen::Ref<const Eigen::VectorXd>& joint_angles, int segment_num) const
{
#ifndef KDL_LESS_1_4_0
  KDL::JntArray kdl_joints;
  EigenToKDL(joint_angles, kdl_joints);

  // run FK solver
  std::vector<KDL::Frame> kdl_pose;
  if (fk_solver_->JntToCart(kdl_joints, kdl_pose, segment_num) < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to calculate FK");
    throw std::runtime_error("KDLFwdKinChain: Failed to calculate forward kinematics.");
  }

  tesseract_common::VectorIsometry3d poses;
  KDLToEigen(kdl_pose, poses);

  return poses;
#else
  throw std::runtime_error("KDLFwdKinChain: Failed to calculate forward kinematics.");
#endif
  UNUSED(joint_angles);
  UNUSED(segment_num);
}

TESSERACT_COMMON_IGNORE_WARNINGS_POP

tesseract_common::VectorIsometry3d
KDLFwdKinChain::calcFwdKinAll(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(joint_angles.size() == numJoints());

  return calcFwdKinHelperAll(joint_angles);
}

Eigen::Isometry3d KDLFwdKinChain::calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(joint_angles.size() == numJoints());

  return calcFwdKinHelper(joint_angles);
}

Eigen::Isometry3d KDLFwdKinChain::calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                             const std::string& link_name) const
{
  assert(checkInitialized());
  assert(joint_angles.size() == numJoints());
  assert(kdl_data_.segment_index.find(link_name) != kdl_data_.segment_index.end());

  int segment_nr = kdl_data_.segment_index.at(link_name);
  return calcFwdKinHelper(joint_angles, segment_nr);
}

bool KDLFwdKinChain::calcJacobianHelper(KDL::Jacobian& jacobian,
                                        const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                        int segment_num) const
{
  KDL::JntArray kdl_joints;
  EigenToKDL(joint_angles, kdl_joints);

  // compute jacobian
  jacobian.resize(static_cast<unsigned>(joint_angles.size()));
  if (jac_solver_->JntToJac(kdl_joints, jacobian, segment_num) < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to calculate jacobian");
    return false;
  }

  return true;
}

Eigen::MatrixXd KDLFwdKinChain::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(joint_angles.size() == numJoints());

  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, joint_angles))
  {
    Eigen::MatrixXd jacobian(6, numJoints());
    KDLToEigen(kdl_jacobian, jacobian);
    return jacobian;
  }

  throw std::runtime_error("KDLFwdKinChain: Failed to calculate jacobian.");
}

Eigen::MatrixXd KDLFwdKinChain::calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                             const std::string& link_name) const
{
  assert(checkInitialized());
  assert(joint_angles.size() == numJoints());

  int segment_nr = kdl_data_.segment_index.at(link_name);
  KDL::Jacobian kdl_jacobian;

  if (calcJacobianHelper(kdl_jacobian, joint_angles, segment_nr))
  {
    Eigen::MatrixXd jacobian(6, numJoints());
    KDLToEigen(kdl_jacobian, jacobian);
    return jacobian;
  }

  throw std::runtime_error("KDLFwdKinChain: Failed to calculate jacobian.");
}

bool KDLFwdKinChain::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != kdl_data_.robot_chain.getNrOfJoints())
  {
    CONSOLE_BRIDGE_logError("Number of joint angles (%d) don't match robot_model (%d)",
                            static_cast<int>(vec.size()),
                            kdl_data_.robot_chain.getNrOfJoints());
    return false;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    if ((vec[i] < kdl_data_.data.limits.joint_limits(i, 0)) || (vec(i) > kdl_data_.data.limits.joint_limits(i, 1)))
    {
      CONSOLE_BRIDGE_logDebug("Joint %s is out-of-range (%g < %g < %g)",
                              kdl_data_.data.joint_names[static_cast<size_t>(i)].c_str(),
                              kdl_data_.data.limits.joint_limits(i, 0),
                              vec(i),
                              kdl_data_.data.limits.joint_limits(i, 1));
      return false;
    }
  }

  return true;
}

const std::vector<std::string>& KDLFwdKinChain::getJointNames() const
{
  assert(checkInitialized());
  return kdl_data_.data.joint_names;
}

const std::vector<std::string>& KDLFwdKinChain::getLinkNames() const
{
  assert(checkInitialized());
  return kdl_data_.data.link_names;
}

const std::vector<std::string>& KDLFwdKinChain::getActiveLinkNames() const
{
  assert(checkInitialized());
  return kdl_data_.data.active_link_names;
}

const tesseract_common::KinematicLimits& KDLFwdKinChain::getLimits() const { return kdl_data_.data.limits; }

void KDLFwdKinChain::setLimits(tesseract_common::KinematicLimits limits)
{
  unsigned int nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.size() != nj ||
      limits.acceleration_limits.size() != nj)
    throw std::runtime_error("Kinematics limits assigned are invalid!");

  kdl_data_.data.limits = std::move(limits);
}

std::vector<Eigen::Index> KDLFwdKinChain::getRedundancyCapableJointIndices() const
{
  return kdl_data_.data.redundancy_indices;
}

tesseract_scene_graph::SceneGraph::ConstPtr KDLFwdKinChain::getSceneGraph() const { return scene_graph_; }

unsigned int KDLFwdKinChain::numJoints() const { return kdl_data_.robot_chain.getNrOfJoints(); }

const std::string& KDLFwdKinChain::getBaseLinkName() const { return kdl_data_.data.base_link_name; }

const std::string& KDLFwdKinChain::getTipLinkName() const { return kdl_data_.data.tip_link_name; }

const std::string& KDLFwdKinChain::getName() const { return name_; }

const std::string& KDLFwdKinChain::getSolverName() const { return solver_name_; }

bool KDLFwdKinChain::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                          const std::vector<std::pair<std::string, std::string>>& chains,
                          std::string name)
{
  initialized_ = false;
  kdl_data_ = KDLChainData();

  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Null pointer to Scene Graph");
    return false;
  }

  scene_graph_ = std::move(scene_graph);
  name_ = std::move(name);

  if (!scene_graph_->getLink(scene_graph_->getRoot()))
  {
    CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
    return false;
  }

  if (!parseSceneGraph(kdl_data_, *scene_graph_, chains))
  {
    CONSOLE_BRIDGE_logError("Failed to parse KDL data from Scene Graph");
    return false;
  }

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_data_.robot_chain);

  initialized_ = true;
  return initialized_;
}

bool KDLFwdKinChain::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                          const std::string& base_link,
                          const std::string& tip_link,
                          std::string name)
{
  std::vector<std::pair<std::string, std::string>> chains;
  chains.push_back(std::make_pair(base_link, tip_link));
  return init(scene_graph, chains, name);
}

bool KDLFwdKinChain::init(const KDLFwdKinChain& kin)
{
  initialized_ = kin.initialized_;
  name_ = kin.name_;
  solver_name_ = kin.solver_name_;
  kdl_data_ = kin.kdl_data_;
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_data_.robot_chain);
  scene_graph_ = kin.scene_graph_;

  return initialized_;
}

bool KDLFwdKinChain::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}

}  // namespace tesseract_kinematics
