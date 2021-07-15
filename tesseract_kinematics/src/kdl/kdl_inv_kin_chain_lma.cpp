/**
 * @file kdl_fwd_kin_chain_lma.cpp
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/segment.hpp>
#include <tesseract_scene_graph/parser/kdl_parser.h>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

InverseKinematics::Ptr KDLInvKinChainLMA::clone() const
{
  auto cloned_invkin = std::make_shared<KDLInvKinChainLMA>();
  cloned_invkin->init(*this);
  return cloned_invkin;
}

bool KDLInvKinChainLMA::update()
{
  if (!init(scene_graph_, orig_data_.base_link_name, orig_data_.tip_link_name, name_))
    return false;

  if (sync_fwd_kin_ != nullptr)
    synchronize(sync_fwd_kin_);

  return true;
}

void KDLInvKinChainLMA::synchronize(ForwardKinematics::ConstPtr fwd_kin)
{
  if (numJoints() != fwd_kin->numJoints())
    throw std::runtime_error("Tried to synchronize kinematics objects with different number of joints!");

  if (!tesseract_common::isIdentical(orig_data_.joint_names, fwd_kin->getJointNames(), false))
    throw std::runtime_error("Tried to synchronize kinematics objects with different joint names!");

  if (!tesseract_common::isIdentical(orig_data_.link_names, fwd_kin->getLinkNames(), false))
    throw std::runtime_error("Tried to synchronize kinematics objects with different link names!");

  if (!tesseract_common::isIdentical(orig_data_.active_link_names, fwd_kin->getActiveLinkNames(), false))
    throw std::runtime_error("Tried to synchronize kinematics objects with different active link names!");

  SynchronizableData local_data;
  local_data.base_link_name = fwd_kin->getBaseLinkName();
  local_data.tip_link_name = fwd_kin->getTipLinkName();
  local_data.joint_names = fwd_kin->getJointNames();
  local_data.link_names = fwd_kin->getLinkNames();
  local_data.active_link_names = fwd_kin->getActiveLinkNames();
  local_data.redundancy_indices = fwd_kin->getRedundancyCapableJointIndices();
  local_data.limits = fwd_kin->getLimits();
  if (kdl_data_.data == local_data)
    return;

  sync_joint_map_.clear();
  const std::vector<std::string>& joint_names = fwd_kin->getJointNames();
  if (orig_data_.joint_names != joint_names)
  {
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      auto it = std::find(orig_data_.joint_names.begin(), orig_data_.joint_names.end(), joint_names[i]);
      Eigen::Index idx = std::distance(orig_data_.joint_names.begin(), it);
      sync_joint_map_.push_back(idx);
    }
  }

  sync_fwd_kin_ = std::move(fwd_kin);
  kdl_data_.data = local_data;
}

bool KDLInvKinChainLMA::isSynchronized() const { return (sync_fwd_kin_ != nullptr); }

IKSolutions KDLInvKinChainLMA::calcInvKinHelper(const Eigen::Isometry3d& pose,
                                                const Eigen::Ref<const Eigen::VectorXd>& seed,
                                                int /*segment_num*/) const
{
  KDL::JntArray kdl_seed, kdl_solution;
  EigenToKDL(seed, kdl_seed);
  kdl_solution.resize(static_cast<unsigned>(seed.size()));
  Eigen::VectorXd solution(seed.size());

  // run IK solver
  // TODO: Need to update to handle seg number. Neet to create an IK solver for each seg.
  KDL::Frame kdl_pose;
  EigenToKDL(pose, kdl_pose);
  int status = ik_solver_->CartToJnt(kdl_seed, kdl_pose, kdl_solution);
  if (status < 0)
  {
    // LCOV_EXCL_START
#ifndef KDL_LESS_1_4_0
    if (status == KDL::ChainIkSolverPos_LMA::E_GRADIENT_JOINTS_TOO_SMALL)
    {
      CONSOLE_BRIDGE_logDebug("KDL LMA Failed to calculate IK, gradient joints are tool small");
    }
    else if (status == KDL::ChainIkSolverPos_LMA::E_INCREMENT_JOINTS_TOO_SMALL)
    {
      CONSOLE_BRIDGE_logDebug("KDL LMA Failed to calculate IK, increment joints are tool small");
    }
    else if (status == KDL::ChainIkSolverPos_LMA::E_MAX_ITERATIONS_EXCEEDED)
    {
      CONSOLE_BRIDGE_logDebug("KDL LMA Failed to calculate IK, max iteration exceeded");
    }
#else
    CONSOLE_BRIDGE_logDebug("KDL LMA Failed to calculate IK");
#endif
    // LCOV_EXCL_STOP
    return IKSolutions();
  }

  KDLToEigen(kdl_solution, solution);

  // Reorder if needed
  if (!sync_joint_map_.empty())
    tesseract_common::reorder(solution, sync_joint_map_);

  IKSolutions solution_set;

  if (tesseract_common::satisfiesPositionLimits(solution, kdl_data_.data.limits.joint_limits))
    solution_set.push_back(solution);

  return solution_set;
}

IKSolutions KDLInvKinChainLMA::calcInvKin(const Eigen::Isometry3d& pose,
                                          const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(checkInitialized());
  return calcInvKinHelper(pose, seed);
}

IKSolutions KDLInvKinChainLMA::calcInvKin(const Eigen::Isometry3d& /*pose*/,
                                          const Eigen::Ref<const Eigen::VectorXd>& /*seed*/,
                                          const std::string& /*link_name*/) const
{
  assert(checkInitialized());

  throw std::runtime_error("This method call is not supported by KDLInvKinChainLMA yet.");
}

bool KDLInvKinChainLMA::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
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

const std::vector<std::string>& KDLInvKinChainLMA::getJointNames() const
{
  assert(checkInitialized());
  return kdl_data_.data.joint_names;
}

const std::vector<std::string>& KDLInvKinChainLMA::getLinkNames() const
{
  assert(checkInitialized());
  return kdl_data_.data.link_names;
}

const std::vector<std::string>& KDLInvKinChainLMA::getActiveLinkNames() const
{
  assert(checkInitialized());
  return kdl_data_.data.active_link_names;
}

const tesseract_common::KinematicLimits& KDLInvKinChainLMA::getLimits() const { return kdl_data_.data.limits; }

void KDLInvKinChainLMA::setLimits(tesseract_common::KinematicLimits limits)
{
  unsigned int nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.size() != nj ||
      limits.acceleration_limits.size() != nj)
    throw std::runtime_error("Kinematics limits assigned are invalid!");

  kdl_data_.data.limits = std::move(limits);
}

std::vector<Eigen::Index> KDLInvKinChainLMA::getRedundancyCapableJointIndices() const
{
  return kdl_data_.data.redundancy_indices;
}

unsigned int KDLInvKinChainLMA::numJoints() const { return kdl_data_.robot_chain.getNrOfJoints(); }

const std::string& KDLInvKinChainLMA::getBaseLinkName() const { return kdl_data_.data.base_link_name; }

const std::string& KDLInvKinChainLMA::getTipLinkName() const { return kdl_data_.data.tip_link_name; }

const std::string& KDLInvKinChainLMA::getName() const { return name_; }

const std::string& KDLInvKinChainLMA::getSolverName() const { return solver_name_; }

tesseract_scene_graph::SceneGraph::ConstPtr KDLInvKinChainLMA::getSceneGraph() const { return scene_graph_; }

bool KDLInvKinChainLMA::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
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

  // Store original sync data
  orig_data_ = kdl_data_.data;

  // Create KDL IK Solver
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_data_.robot_chain);

  initialized_ = true;
  return initialized_;
}

bool KDLInvKinChainLMA::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                             const std::string& base_link,
                             const std::string& tip_link,
                             std::string name)
{
  std::vector<std::pair<std::string, std::string>> chains;
  chains.push_back(std::make_pair(base_link, tip_link));
  return init(scene_graph, chains, name);
}

bool KDLInvKinChainLMA::init(const KDLInvKinChainLMA& kin)
{
  initialized_ = kin.initialized_;
  name_ = kin.name_;
  solver_name_ = kin.solver_name_;
  kdl_data_ = kin.kdl_data_;
  orig_data_ = kin.orig_data_;
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_data_.robot_chain);
  scene_graph_ = kin.scene_graph_;
  sync_fwd_kin_ = kin.sync_fwd_kin_;
  sync_joint_map_ = kin.sync_joint_map_;

  return initialized_;
}

bool KDLInvKinChainLMA::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}
}  // namespace tesseract_kinematics
