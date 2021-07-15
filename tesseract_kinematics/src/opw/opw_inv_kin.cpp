/**
 * @file opw_inv_kin.cpp
 * @brief Tesseract OPW Inverse kinematics implementation.
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
#include <stdexcept>
#include <console_bridge/console.h>
#include <opw_kinematics/opw_kinematics.h>
#include <opw_kinematics/opw_utilities.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
InverseKinematics::Ptr OPWInvKin::clone() const
{
  auto cloned_invkin = std::make_shared<OPWInvKin>();
  cloned_invkin->init(*this);
  return cloned_invkin;
}

bool OPWInvKin::update()
{
  if (!init(name_,
            params_,
            orig_data_.base_link_name,
            orig_data_.tip_link_name,
            orig_data_.joint_names,
            orig_data_.link_names,
            orig_data_.active_link_names,
            orig_data_.limits))
    return false;

  if (sync_fwd_kin_ != nullptr)
    synchronize(sync_fwd_kin_);

  return true;
}

void OPWInvKin::synchronize(ForwardKinematics::ConstPtr fwd_kin)
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
  if (data_ == local_data)
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
  data_ = local_data;
}

bool OPWInvKin::isSynchronized() const { return (sync_fwd_kin_ != nullptr); }

IKSolutions OPWInvKin::calcInvKin(const Eigen::Isometry3d& pose,
                                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const
{
  opw_kinematics::Solutions<double> sols = opw_kinematics::inverse(params_, pose);

  // Check the output
  IKSolutions solution_set;
  solution_set.reserve(sols.size());
  for (auto& sol : sols)
  {
    if (opw_kinematics::isValid<double>(sol))
    {
      Eigen::Map<Eigen::VectorXd> eigen_sol(sol.data(), static_cast<Eigen::Index>(sol.size()));

      // Harmonize between [-PI, PI]
      harmonizeTowardZero<double>(eigen_sol);  // Modifies 'sol' in place

      // Reorder if needed
      if (!sync_joint_map_.empty())
        tesseract_common::reorder(eigen_sol, sync_joint_map_);

      // Add solution
      if (tesseract_common::satisfiesPositionLimits(eigen_sol, data_.limits.joint_limits))
        solution_set.push_back(eigen_sol);
    }
  }

  return solution_set;
}

IKSolutions OPWInvKin::calcInvKin(const Eigen::Isometry3d& /*pose*/,
                                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/,
                                  const std::string& /*link_name*/) const
{
  throw std::runtime_error("OPWInvKin::calcInvKin(const Eigen::Isometry3d&, const Eigen::Ref<const Eigen::VectorXd>&, "
                           "const std::string&) Not Supported!");
}

bool OPWInvKin::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != numJoints())
  {
    CONSOLE_BRIDGE_logError(
        "Number of joint angles (%d) don't match robot_model (%d)", static_cast<int>(vec.size()), numJoints());
    return false;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    if ((vec[i] < data_.limits.joint_limits(i, 0)) || (vec(i) > data_.limits.joint_limits(i, 1)))
    {
      CONSOLE_BRIDGE_logDebug("Joint %s is out-of-range (%g < %g < %g)",
                              data_.joint_names[static_cast<size_t>(i)].c_str(),
                              data_.limits.joint_limits(i, 0),
                              vec(i),
                              data_.limits.joint_limits(i, 1));
      return false;
    }
  }

  return true;
}

unsigned int OPWInvKin::numJoints() const { return 6; }

const std::vector<std::string>& OPWInvKin::getJointNames() const { return data_.joint_names; }
const std::vector<std::string>& OPWInvKin::getLinkNames() const { return data_.link_names; }
const std::vector<std::string>& OPWInvKin::getActiveLinkNames() const { return data_.active_link_names; }
const tesseract_common::KinematicLimits& OPWInvKin::getLimits() const { return data_.limits; }

void OPWInvKin::setLimits(tesseract_common::KinematicLimits limits)
{
  unsigned int nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.size() != nj ||
      limits.acceleration_limits.size() != nj)
    throw std::runtime_error("Kinematics limits assigned are invalid!");

  data_.limits = std::move(limits);
}

std::vector<Eigen::Index> OPWInvKin::getRedundancyCapableJointIndices() const { return data_.redundancy_indices; }

const std::string& OPWInvKin::getBaseLinkName() const { return data_.base_link_name; }
const std::string& OPWInvKin::getTipLinkName() const { return data_.tip_link_name; }
const std::string& OPWInvKin::getName() const { return name_; }
const std::string& OPWInvKin::getSolverName() const { return solver_name_; }

bool OPWInvKin::init(std::string name,
                     opw_kinematics::Parameters<double> params,
                     std::string base_link_name,
                     std::string tip_link_name,
                     std::vector<std::string> joint_names,
                     std::vector<std::string> link_names,
                     std::vector<std::string> active_link_names,
                     tesseract_common::KinematicLimits limits)
{
  assert(joint_names.size() == 6);

  name_ = std::move(name);
  params_ = params;
  data_.clear();
  data_.base_link_name = std::move(base_link_name);
  data_.tip_link_name = std::move(tip_link_name);
  data_.joint_names = std::move(joint_names);
  data_.link_names = std::move(link_names);
  data_.active_link_names = std::move(active_link_names);
  data_.limits = std::move(limits);
  data_.redundancy_indices = { 0, 1, 2, 3, 4, 5 };
  orig_data_ = data_;
  initialized_ = true;

  return initialized_;
}

bool OPWInvKin::init(const OPWInvKin& kin)
{
  initialized_ = kin.initialized_;
  sync_fwd_kin_ = kin.sync_fwd_kin_;
  sync_joint_map_ = kin.sync_joint_map_;
  name_ = kin.name_;
  params_ = kin.params_;
  solver_name_ = kin.solver_name_;
  data_ = kin.data_;
  orig_data_ = kin.orig_data_;

  return initialized_;
}

bool OPWInvKin::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}

}  // namespace tesseract_kinematics
