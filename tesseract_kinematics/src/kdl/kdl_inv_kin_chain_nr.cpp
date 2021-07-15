/**
 * @file kdl_fwd_kin_chain_nr.cpp
 * @brief Tesseract KDL inverse kinematics chain Newton-Raphson implementation.
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

#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

InverseKinematics::Ptr KDLInvKinChainNR::clone() const
{
  auto cloned_invkin = std::make_shared<KDLInvKinChainNR>();
  cloned_invkin->init(*this);
  return cloned_invkin;
}

bool KDLInvKinChainNR::update()
{
  if (!init(scene_graph_, orig_data_.base_link_name, orig_data_.tip_link_name, name_))
    return false;

  if (sync_fwd_kin_ != nullptr)
    synchronize(sync_fwd_kin_);

  return true;
}

void KDLInvKinChainNR::synchronize(ForwardKinematics::ConstPtr fwd_kin)
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

bool KDLInvKinChainNR::isSynchronized() const { return (sync_fwd_kin_ != nullptr); }

IKSolutions KDLInvKinChainNR::calcInvKinHelper(const Eigen::Isometry3d& pose,
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
    if (status == KDL::ChainIkSolverPos_NR::E_DEGRADED)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, solution converged to <eps in maxiter, but solution is "
                              "degraded in quality (e.g. pseudo-inverse in iksolver is singular)");
    }
    else if (status == KDL::ChainIkSolverPos_NR::E_IKSOLVER_FAILED)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, velocity solver failed");
    }
    else if (status == KDL::ChainIkSolverPos_NR::E_NO_CONVERGE)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, no solution found");
    }
#ifndef KDL_LESS_1_4_0
    else if (status == KDL::ChainIkSolverPos_NR::E_MAX_ITERATIONS_EXCEEDED)
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK, max iteration exceeded");
    }
#endif
    else
    {
      CONSOLE_BRIDGE_logDebug("KDL NR Failed to calculate IK");
    }
    // LCOV_EXCL_STOP
    return IKSolutions();
  }

  KDLToEigen(kdl_solution, solution);

  IKSolutions solution_set;
  if (tesseract_common::satisfiesPositionLimits(solution, kdl_data_.data.limits.joint_limits))
    solution_set.push_back(solution);

  return solution_set;
}

IKSolutions KDLInvKinChainNR::calcInvKin(const Eigen::Isometry3d& pose,
                                         const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(checkInitialized());
  return calcInvKinHelper(pose, seed);
}

IKSolutions KDLInvKinChainNR::calcInvKin(const Eigen::Isometry3d& /*pose*/,
                                         const Eigen::Ref<const Eigen::VectorXd>& /*seed*/,
                                         const std::string& /*link_name*/) const
{
  assert(checkInitialized());

  throw std::runtime_error("This method call is not supported by KDLInvKinChainNR yet.");
}

bool KDLInvKinChainNR::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
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

const std::vector<std::string>& KDLInvKinChainNR::getJointNames() const
{
  assert(checkInitialized());
  return kdl_data_.data.joint_names;
}

const std::vector<std::string>& KDLInvKinChainNR::getLinkNames() const
{
  assert(checkInitialized());
  return kdl_data_.data.link_names;
}

const std::vector<std::string>& KDLInvKinChainNR::getActiveLinkNames() const
{
  assert(checkInitialized());
  return kdl_data_.data.active_link_names;
}

const tesseract_common::KinematicLimits& KDLInvKinChainNR::getLimits() const { return kdl_data_.data.limits; }

void KDLInvKinChainNR::setLimits(tesseract_common::KinematicLimits limits)
{
  unsigned int nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.size() != nj ||
      limits.acceleration_limits.size() != nj)
    throw std::runtime_error("Kinematics limits assigned are invalid!");

  kdl_data_.data.limits = std::move(limits);
}

std::vector<Eigen::Index> KDLInvKinChainNR::getRedundancyCapableJointIndices() const
{
  return kdl_data_.data.redundancy_indices;
}

unsigned int KDLInvKinChainNR::numJoints() const { return kdl_data_.robot_chain.getNrOfJoints(); }

const std::string& KDLInvKinChainNR::getBaseLinkName() const { return kdl_data_.data.base_link_name; }

const std::string& KDLInvKinChainNR::getTipLinkName() const { return kdl_data_.data.tip_link_name; }

const std::string& KDLInvKinChainNR::getName() const { return name_; }

const std::string& KDLInvKinChainNR::getSolverName() const { return solver_name_; }

tesseract_scene_graph::SceneGraph::ConstPtr KDLInvKinChainNR::getSceneGraph() const { return scene_graph_; }

bool KDLInvKinChainNR::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
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

  scene_graph_ = scene_graph;
  name_ = name;

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

  // Create KDL FK and IK Solver
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_data_.robot_chain);
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR>(kdl_data_.robot_chain, *fk_solver_, *ik_vel_solver_);

  initialized_ = true;
  return initialized_;
}

bool KDLInvKinChainNR::init(const tesseract_scene_graph::SceneGraph::ConstPtr& scene_graph,
                            const std::string& base_link,
                            const std::string& tip_link,
                            const std::string& name)
{
  std::vector<std::pair<std::string, std::string>> chains;
  chains.push_back(std::make_pair(base_link, tip_link));
  return init(scene_graph, chains, name);
}

bool KDLInvKinChainNR::init(const KDLInvKinChainNR& kin)
{
  initialized_ = kin.initialized_;
  name_ = kin.name_;
  solver_name_ = kin.solver_name_;
  kdl_data_ = kin.kdl_data_;
  orig_data_ = kin.orig_data_;
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_data_.robot_chain);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_data_.robot_chain);
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR>(kdl_data_.robot_chain, *fk_solver_, *ik_vel_solver_);
  scene_graph_ = kin.scene_graph_;
  sync_fwd_kin_ = kin.sync_fwd_kin_;
  sync_joint_map_ = kin.sync_joint_map_;

  return initialized_;
}

bool KDLInvKinChainNR::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}

}  // namespace tesseract_kinematics
