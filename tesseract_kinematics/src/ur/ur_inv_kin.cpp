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
#include <tesseract_kinematics/ur/ur_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
namespace
{
const double ZERO_THRESH = 0.00000001;
int SIGN(double x) { return (x > 0) - (x < 0); }
const double PI = M_PI;
}  // namespace

int inverse(const Eigen::Isometry3d& T, const URParameters& params, double* q_sols, double q6_des)
{
  int num_sols = 0;
  double T02 = T(0, 2);
  double T00 = T(0, 0);
  double T01 = T(0, 1);
  double T03 = T(0, 3);
  double T12 = T(1, 2);
  double T10 = T(1, 0);
  double T11 = T(1, 1);
  double T13 = T(1, 3);
  double T22 = T(2, 2);
  double T20 = T(2, 0);
  double T21 = T(2, 1);
  double T23 = T(2, 3);

  ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
  double q1[2];
  {
    double A = params.d6 * T12 - T13;
    double B = params.d6 * T02 - T03;
    double R = A * A + B * B;
    if (fabs(A) < ZERO_THRESH)
    {
      double div;
      if (fabs(fabs(params.d4) - fabs(B)) < ZERO_THRESH)
        div = -SIGN(params.d4) * SIGN(B);
      else
        div = -params.d4 / B;
      double arcsin = asin(div);
      if (fabs(arcsin) < ZERO_THRESH)
        arcsin = 0.0;
      if (arcsin < 0.0)
        q1[0] = arcsin + 2.0 * PI;
      else
        q1[0] = arcsin;
      q1[1] = PI - arcsin;
    }
    else if (fabs(B) < ZERO_THRESH)
    {
      double div;
      if (fabs(fabs(params.d4) - fabs(A)) < ZERO_THRESH)
        div = SIGN(params.d4) * SIGN(A);
      else
        div = params.d4 / A;
      double arccos = acos(div);
      q1[0] = arccos;
      q1[1] = 2.0 * PI - arccos;
    }
    else if (params.d4 * params.d4 > R)
    {
      return num_sols;
    }
    else
    {
      double arccos = acos(params.d4 / sqrt(R));
      double arctan = atan2(-B, A);
      double pos = arccos + arctan;
      double neg = -arccos + arctan;
      if (fabs(pos) < ZERO_THRESH)
        pos = 0.0;
      if (fabs(neg) < ZERO_THRESH)
        neg = 0.0;
      if (pos >= 0.0)
        q1[0] = pos;
      else
        q1[0] = 2.0 * PI + pos;
      if (neg >= 0.0)
        q1[1] = neg;
      else
        q1[1] = 2.0 * PI + neg;
    }
  }
  ////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
  double q5[2][2];
  {
    for (int i = 0; i < 2; i++)
    {
      double numer = (T03 * sin(q1[i]) - T13 * cos(q1[i]) - params.d4);
      double div;
      if (fabs(fabs(numer) - fabs(params.d6)) < ZERO_THRESH)
        div = SIGN(numer) * SIGN(params.d6);
      else
        div = numer / params.d6;
      double arccos = acos(div);
      q5[i][0] = arccos;
      q5[i][1] = 2.0 * PI - arccos;
    }
  }
  ////////////////////////////////////////////////////////////////////////////////

  {
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        double c1 = cos(q1[i]), s1 = sin(q1[i]);
        double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
        double q6;
        ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
        if (fabs(s5) < ZERO_THRESH)
          q6 = q6_des;
        else
        {
          q6 = atan2(SIGN(s5) * -(T01 * s1 - T11 * c1), SIGN(s5) * (T00 * s1 - T10 * c1));
          if (fabs(q6) < ZERO_THRESH)
            q6 = 0.0;
          if (q6 < 0.0)
            q6 += 2.0 * PI;
        }
        ////////////////////////////////////////////////////////////////////////////////

        double q2[2], q3[2], q4[2];
        ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
        double c6 = cos(q6), s6 = sin(q6);
        double x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1));
        double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
        double p13x = params.d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) -
                      params.d6 * (T02 * c1 + T12 * s1) + T03 * c1 + T13 * s1;
        double p13y = T23 - params.d1 - params.d6 * T22 + params.d5 * (T21 * c6 + T20 * s6);

        double c3 =
            (p13x * p13x + p13y * p13y - params.a2 * params.a2 - params.a3 * params.a3) / (2.0 * params.a2 * params.a3);
        if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
          c3 = SIGN(c3);
        else if (fabs(c3) > 1.0)
        {
          // TODO NO SOLUTION
          continue;
        }
        double arccos = acos(c3);
        q3[0] = arccos;
        q3[1] = 2.0 * PI - arccos;
        double denom = params.a2 * params.a2 + params.a3 * params.a3 + 2 * params.a2 * params.a3 * c3;
        double s3 = sin(arccos);
        double A = (params.a2 + params.a3 * c3), B = params.a3 * s3;
        q2[0] = atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom);
        q2[1] = atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);
        double c23_0 = cos(q2[0] + q3[0]);
        double s23_0 = sin(q2[0] + q3[0]);
        double c23_1 = cos(q2[1] + q3[1]);
        double s23_1 = sin(q2[1] + q3[1]);
        q4[0] = atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
        q4[1] = atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1);
        ////////////////////////////////////////////////////////////////////////////////
        for (int k = 0; k < 2; k++)
        {
          if (fabs(q2[k]) < ZERO_THRESH)
            q2[k] = 0.0;
          else if (q2[k] < 0.0)
            q2[k] += 2.0 * PI;
          if (fabs(q4[k]) < ZERO_THRESH)
            q4[k] = 0.0;
          else if (q4[k] < 0.0)
            q4[k] += 2.0 * PI;
          q_sols[num_sols * 6 + 0] = q1[i];
          q_sols[num_sols * 6 + 1] = q2[k];
          q_sols[num_sols * 6 + 2] = q3[k];
          q_sols[num_sols * 6 + 3] = q4[k];
          q_sols[num_sols * 6 + 4] = q5[i][j];
          q_sols[num_sols * 6 + 5] = q6;
          num_sols++;
        }
      }
    }
  }
  return num_sols;
}

InverseKinematics::Ptr URInvKin::clone() const
{
  auto cloned_invkin = std::make_shared<URInvKin>();
  cloned_invkin->init(*this);
  return cloned_invkin;
}

bool URInvKin::update()
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

void URInvKin::synchronize(ForwardKinematics::ConstPtr fwd_kin)
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

bool URInvKin::isSynchronized() const { return (sync_fwd_kin_ != nullptr); }

IKSolutions URInvKin::calcInvKin(const Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const
{
  Eigen::Isometry3d base_offset = Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
  Eigen::Isometry3d corrected_pose = base_offset.inverse() * pose;

  // Do the analytic IK
  std::array<std::array<double, 6>, 8> sols;  // maximum of 8 IK solutions
  std::size_t num_sols = static_cast<std::size_t>(inverse(corrected_pose, params_, sols[0].data(), 0));

  // Check the output
  IKSolutions solution_set;
  solution_set.reserve(num_sols);
  for (std::size_t i = 0; i < num_sols; ++i)
  {
    Eigen::Map<Eigen::VectorXd> eigen_sol(sols[i].data(), static_cast<Eigen::Index>(sols[i].size()));

    // Harmonize between [-PI, PI]
    harmonizeTowardZero<double>(eigen_sol);  // Modifies 'sol' in place

    // Reorder if needed
    if (!sync_joint_map_.empty())
      tesseract_common::reorder(eigen_sol, sync_joint_map_);

    // Add solution
    solution_set.push_back(eigen_sol);
  }

  return solution_set;
}

IKSolutions URInvKin::calcInvKin(const Eigen::Isometry3d& pose,
                                 const Eigen::Ref<const Eigen::VectorXd>& seed,
                                 const std::string& link_name) const
{
  if (link_name == data_.tip_link_name)
    return calcInvKin(pose, seed);

  throw std::runtime_error("UR5InvKin::calcInvKin(const Eigen::Isometry3d&, const Eigen::Ref<const Eigen::VectorXd>&, "
                           "const std::string&) Not Supported!");
}

bool URInvKin::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
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

unsigned int URInvKin::numJoints() const { return 6; }

const std::vector<std::string>& URInvKin::getJointNames() const { return data_.joint_names; }
const std::vector<std::string>& URInvKin::getLinkNames() const { return data_.link_names; }
const std::vector<std::string>& URInvKin::getActiveLinkNames() const { return data_.active_link_names; }
const tesseract_common::KinematicLimits& URInvKin::getLimits() const { return data_.limits; }

void URInvKin::setLimits(tesseract_common::KinematicLimits limits)
{
  unsigned int nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.size() != nj ||
      limits.acceleration_limits.size() != nj)
    throw std::runtime_error("Kinematics limits assigned are invalid!");

  data_.limits = std::move(limits);
}

std::vector<Eigen::Index> URInvKin::getRedundancyCapableJointIndices() const { return data_.redundancy_indices; }

const std::string& URInvKin::getBaseLinkName() const { return data_.base_link_name; }
const std::string& URInvKin::getTipLinkName() const { return data_.tip_link_name; }
const std::string& URInvKin::getName() const { return name_; }
const std::string& URInvKin::getSolverName() const { return solver_name_; }

bool URInvKin::init(std::string name,
                    URParameters params,
                    std::string base_link_name,
                    std::string tip_link_name,
                    std::vector<std::string> joint_names,
                    std::vector<std::string> link_names,
                    std::vector<std::string> active_link_names,
                    tesseract_common::KinematicLimits limits)
{
  assert(joint_names.size() == 6);

  name_ = std::move(name);
  params_ = std::move(params);
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

bool URInvKin::init(const URInvKin& kin)
{
  initialized_ = kin.initialized_;
  sync_fwd_kin_ = kin.sync_fwd_kin_;
  sync_joint_map_ = kin.sync_joint_map_;
  name_ = kin.name_;
  solver_name_ = kin.solver_name_;
  params_ = kin.params_;
  data_ = kin.data_;
  orig_data_ = kin.orig_data_;

  return initialized_;
}

bool URInvKin::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}

}  // namespace tesseract_kinematics
