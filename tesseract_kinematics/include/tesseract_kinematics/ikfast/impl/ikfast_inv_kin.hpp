/**
 * @file ikfast_inv_kin.hpp
 * @brief Tesseract IKFast Inverse kinematics Wrapper Implementation
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
#ifndef TESSERACT_KINEMATICS_IMPL_IKFAST_INV_KIN_HPP
#define TESSERACT_KINEMATICS_IMPL_IKFAST_INV_KIN_HPP

#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <console_bridge/console.h>
#include <tesseract_kinematics/ikfast/external/ikfast.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/ikfast/ikfast_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
inline InverseKinematics::Ptr IKFastInvKin::clone() const
{
  auto cloned_invkin = std::make_shared<IKFastInvKin>();
  cloned_invkin->init(*this);
  return cloned_invkin;
}

inline bool IKFastInvKin::update()
{
  if (!init(name_,
            orig_data_.base_link_name,
            orig_data_.tip_link_name,
            orig_data_.joint_names,
            orig_data_.link_names,
            orig_data_.active_link_names,
            orig_data_.limits,
            orig_data_.redundancy_indices))
    return false;

  if (sync_fwd_kin_ != nullptr)
    synchronize(sync_fwd_kin_);

  return true;
}

inline void IKFastInvKin::synchronize(ForwardKinematics::ConstPtr fwd_kin)
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
    for (const auto& joint_name : joint_names)
    {
      auto it = std::find(orig_data_.joint_names.begin(), orig_data_.joint_names.end(), joint_name);
      Eigen::Index idx = std::distance(orig_data_.joint_names.begin(), it);
      sync_joint_map_.push_back(idx);
    }
  }

  sync_fwd_kin_ = std::move(fwd_kin);
  data_ = local_data;
}

inline bool IKFastInvKin::isSynchronized() const { return (sync_fwd_kin_ != nullptr); }

inline IKSolutions IKFastInvKin::calcInvKin(const Eigen::Isometry3d& pose,
                                            const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const
{
  // Convert to ikfast data type
  Eigen::Transform<IkReal, 3, Eigen::Isometry> ikfast_tcp = pose.cast<IkReal>();

  // Decompose
  const Eigen::Matrix<IkReal, 3, 1> translation = ikfast_tcp.translation();

  // Note the row major ordering here: IkFast expects the matrix in r00, r01, r02, ..., r11, r12, r13
  // ordering
  const Eigen::Matrix<IkReal, 3, 3, Eigen::RowMajor> rotation = ikfast_tcp.rotation();

  // Call IK (TODO: Make a better solution list class? One that uses vector instead of list)
  ikfast::IkSolutionList<IkReal> ikfast_solution_set;
  ComputeIk(translation.data(), rotation.data(), nullptr, ikfast_solution_set);

  // Unpack the solutions into the output vector
  const auto n_sols = ikfast_solution_set.GetNumSolutions();
  int ikfast_dof = static_cast<int>(numJoints());

  std::vector<IkReal> ikfast_output;
  ikfast_output.resize(n_sols * static_cast<std::size_t>(ikfast_dof));

  for (std::size_t i = 0; i < n_sols; ++i)
  {
    // This actually walks the list EVERY time from the start of i.
    const auto& sol = ikfast_solution_set.GetSolution(i);
    auto* out = ikfast_output.data() + i * static_cast<std::size_t>(ikfast_dof);
    sol.GetSolution(out, nullptr);
  }

  std::vector<double> sols;
  sols.insert(end(sols), std::make_move_iterator(ikfast_output.begin()), std::make_move_iterator(ikfast_output.end()));

  // Check the output
  int num_sol = static_cast<int>(sols.size()) / ikfast_dof;
  IKSolutions solution_set;
  solution_set.reserve(sols.size());
  for (int i = 0; i < num_sol; i++)
  {
    Eigen::Map<Eigen::VectorXd> eigen_sol(sols.data() + ikfast_dof * i, static_cast<Eigen::Index>(ikfast_dof));
    if (eigen_sol.array().allFinite())
    {
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

inline IKSolutions IKFastInvKin::calcInvKin(const Eigen::Isometry3d& pose,
                                            const Eigen::Ref<const Eigen::VectorXd>& seed,
                                            const std::string& link_name) const
{
  if (link_name == data_.tip_link_name)
    return calcInvKin(pose, seed);

  throw std::runtime_error("IKFastInvKin::calcInvKin(Eigen::VectorXd&, const Eigen::Isometry3d&, const "
                           "Eigen::Ref<const Eigen::VectorXd>&, const std::string&) Not Supported!");
}

inline bool IKFastInvKin::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != numJoints())
  {
    CONSOLE_BRIDGE_logError(
        "Number of joint angles (%d) don't match robot_model (%d)", static_cast<int>(vec.size()), numJoints());
    return false;
  }

  return (tesseract_common::satisfiesPositionLimits(vec, data_.limits.joint_limits));
}

inline unsigned int IKFastInvKin::numJoints() const { return static_cast<unsigned int>(GetNumJoints()); }

inline bool IKFastInvKin::init(std::string name,
                               std::string base_link_name,
                               std::string tip_link_name,
                               std::vector<std::string> joint_names,
                               std::vector<std::string> link_names,
                               std::vector<std::string> active_link_names,
                               tesseract_common::KinematicLimits limits,
                               std::vector<Eigen::Index> redundancy_indices)
{
  name_ = std::move(name);
  data_.clear();
  data_.base_link_name = std::move(base_link_name);
  data_.tip_link_name = std::move(tip_link_name);
  data_.joint_names = std::move(joint_names);
  data_.link_names = std::move(link_names);
  data_.active_link_names = std::move(active_link_names);
  data_.limits = std::move(limits);
  data_.redundancy_indices = std::move(redundancy_indices);
  orig_data_ = data_;
  initialized_ = true;

  return initialized_;
}

inline bool IKFastInvKin::init(const IKFastInvKin& kin)
{
  initialized_ = kin.initialized_;
  sync_fwd_kin_ = kin.sync_fwd_kin_;
  sync_joint_map_ = kin.sync_joint_map_;
  name_ = kin.name_;
  solver_name_ = kin.solver_name_;
  data_ = kin.data_;
  orig_data_ = kin.orig_data_;

  return initialized_;
}

inline const std::vector<std::string>& IKFastInvKin::getJointNames() const { return data_.joint_names; }
inline const std::vector<std::string>& IKFastInvKin::getLinkNames() const { return data_.link_names; }
inline const std::vector<std::string>& IKFastInvKin::getActiveLinkNames() const { return data_.active_link_names; }
inline const tesseract_common::KinematicLimits& IKFastInvKin::getLimits() const { return data_.limits; }

inline void IKFastInvKin::setLimits(tesseract_common::KinematicLimits limits)
{
  unsigned int nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.size() != nj ||
      limits.acceleration_limits.size() != nj)
    throw std::runtime_error("Kinematics limits assigned are invalid!");

  data_.limits = std::move(limits);
}
inline std::vector<Eigen::Index> IKFastInvKin::getRedundancyCapableJointIndices() const
{
  return data_.redundancy_indices;
}
inline const std::string& IKFastInvKin::getBaseLinkName() const { return data_.base_link_name; }
inline const std::string& IKFastInvKin::getTipLinkName() const { return data_.tip_link_name; }
inline const std::string& IKFastInvKin::getName() const { return name_; }
inline const std::string& IKFastInvKin::getSolverName() const { return solver_name_; }

inline bool IKFastInvKin::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_IMPL_IKFAST_INV_KIN_HPP
