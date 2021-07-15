/**
 * @file kin_test_suite.h
 * @brief Tesseract kinematics test suite
 *
 * @author Levi Armstrong
 * @date Feb 4, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_KIN_TEST_SUITE_H
#define TESSERACT_KINEMATICS_KIN_TEST_SUITE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
#include <tesseract_urdf/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/forward_kinematics_factory.h>
#include <tesseract_kinematics/core/inverse_kinematics_factory.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/core/types.h>

namespace tesseract_kinematics
{
namespace test_suite
{
inline std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;  // "file://" + package_path + mod_url;
  }

  return mod_url;
}

inline tesseract_scene_graph::SceneGraph::Ptr getSceneGraphIIWA()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

inline tesseract_scene_graph::SceneGraph::Ptr getSceneGraphABBExternalPositioner()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400_external_positioner.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);

  return tesseract_urdf::parseURDFFile(path, locator);
}

inline tesseract_scene_graph::SceneGraph::Ptr getSceneGraphABBOnPositioner()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400_on_positioner.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);

  return tesseract_urdf::parseURDFFile(path, locator);
}

inline tesseract_scene_graph::SceneGraph::Ptr getSceneGraphABB()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);

  return tesseract_urdf::parseURDFFile(path, locator);
}

inline tesseract_scene_graph::SceneGraph::Ptr getSceneGraphUR(const tesseract_kinematics::URParameters& params)
{
  using namespace tesseract_scene_graph;

  auto sg = std::make_shared<SceneGraph>("universal_robot");
  sg->addLink(Link("base_link"));
  sg->addLink(Link("shoulder_link"));
  sg->addLink(Link("upper_arm_link"));
  sg->addLink(Link("forearm_link"));
  sg->addLink(Link("wrist_1_link"));
  sg->addLink(Link("wrist_2_link"));
  sg->addLink(Link("wrist_3_link"));
  sg->addLink(Link("ee_link"));
  sg->addLink(Link("tool0"));

  {
    Joint j("shoulder_pan_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "base_link";
    j.child_link_name = "shoulder_link";
    j.axis = Eigen::Vector3d::UnitZ();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, params.d1);
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("shoulder_lift_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "shoulder_link";
    j.child_link_name = "upper_arm_link";
    j.axis = Eigen::Vector3d::UnitY();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0.220941, 0);
    j.parent_to_joint_origin_transform =
        j.parent_to_joint_origin_transform * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("elbow_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "upper_arm_link";
    j.child_link_name = "forearm_link";
    j.axis = Eigen::Vector3d::UnitY();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, -0.1719, -params.a2);
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("wrist_1_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "forearm_link";
    j.child_link_name = "wrist_1_link";
    j.axis = Eigen::Vector3d::UnitY();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, -params.a3);
    j.parent_to_joint_origin_transform =
        j.parent_to_joint_origin_transform * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("wrist_2_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "wrist_1_link";
    j.child_link_name = "wrist_2_link";
    j.axis = Eigen::Vector3d::UnitZ();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, params.d4 + 0.1719 - 0.220941, 0);
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("wrist_3_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "wrist_2_link";
    j.child_link_name = "wrist_3_link";
    j.axis = Eigen::Vector3d::UnitY();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, params.d5);
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("ee_fixed_joint");
    j.type = JointType::FIXED;
    j.parent_link_name = "wrist_3_link";
    j.child_link_name = "ee_link";
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, params.d6, 0);
    j.parent_to_joint_origin_transform =
        j.parent_to_joint_origin_transform * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
    sg->addJoint(j);
  }

  {
    Joint j("wrist_3_link-tool0_fixed_joint");
    j.type = JointType::FIXED;
    j.parent_link_name = "wrist_3_link";
    j.child_link_name = "tool0";
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, params.d6, 0);
    j.parent_to_joint_origin_transform =
        j.parent_to_joint_origin_transform * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX());
    sg->addJoint(j);
  }

  return sg;
}

inline tesseract_common::KinematicLimits getTargetLimits(const tesseract_scene_graph::SceneGraph::ConstPtr& scene_graph,
                                                         const std::vector<std::string>& joint_names)
{
  auto s = static_cast<Eigen::Index>(joint_names.size());

  tesseract_common::KinematicLimits limits;
  limits.joint_limits.resize(s, 2);
  limits.velocity_limits.resize(s);
  limits.acceleration_limits.resize(s);

  for (Eigen::Index i = 0; i < s; ++i)
  {
    auto joint = scene_graph->getJoint(joint_names[static_cast<std::size_t>(i)]);
    limits.joint_limits(i, 0) = joint->limits->lower;
    limits.joint_limits(i, 1) = joint->limits->upper;
    limits.velocity_limits(i) = joint->limits->velocity;
    limits.acceleration_limits(i) = joint->limits->acceleration;
  }

  return limits;
}

/**
 * @brief Run a kinematic jacobian test
 * @param kin The kinematics object
 * @param jvals The joint values to calculate the jacobian about
 * @param link_name Name of link to calculate jacobian. If empty it will use the function that does not require link
 * name
 * @param link_point Is expressed in the same base frame of the jacobian and is a vector from the old point to the new
 * point.
 * @param change_base The transform from the desired frame to the current base frame of the jacobian
 */
inline void runJacobianTest(tesseract_kinematics::ForwardKinematics& kin,
                            const Eigen::VectorXd& jvals,
                            const std::string& link_name,
                            const Eigen::Vector3d& link_point,
                            const Eigen::Isometry3d& change_base)
{
  Eigen::MatrixXd jacobian, numerical_jacobian;
  Eigen::Isometry3d pose;

  jacobian.resize(6, kin.numJoints());
  if (link_name.empty())
  {
    pose = kin.calcFwdKin(jvals);
    jacobian = kin.calcJacobian(jvals);
  }
  else
  {
    pose = kin.calcFwdKin(jvals, link_name);
    jacobian = kin.calcJacobian(jvals, link_name);
  }
  tesseract_kinematics::jacobianChangeBase(jacobian, change_base);
  tesseract_kinematics::jacobianChangeRefPoint(jacobian, (change_base * pose).linear() * link_point);

  numerical_jacobian.resize(6, kin.numJoints());
  if (link_name.empty())
    tesseract_kinematics::numericalJacobian(
        numerical_jacobian, change_base, kin, jvals, kin.getTipLinkName(), link_point);
  else
    tesseract_kinematics::numericalJacobian(numerical_jacobian, change_base, kin, jvals, link_name, link_point);

  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < static_cast<int>(kin.numJoints()); ++j)
      EXPECT_NEAR(numerical_jacobian(i, j), jacobian(i, j), 1e-3);
}

/**
 * @brief Run kinematic limits test
 * @param limits The limits to check
 * @param target_limits The target to compare to
 */
inline void runKinJointLimitsTest(const tesseract_common::KinematicLimits& limits,
                                  const tesseract_common::KinematicLimits& target_limits)
{
  //////////////////////////////////////////////////////////////////
  // Test forward kinematics joint limits
  //////////////////////////////////////////////////////////////////
  EXPECT_EQ(limits.joint_limits.rows(), target_limits.joint_limits.rows());
  EXPECT_EQ(limits.velocity_limits.rows(), target_limits.velocity_limits.rows());
  EXPECT_EQ(limits.acceleration_limits.rows(), target_limits.acceleration_limits.rows());

  // Check limits
  for (Eigen::Index i = 0; i < limits.joint_limits.rows(); ++i)
  {
    EXPECT_NEAR(limits.joint_limits(i, 0), target_limits.joint_limits(i, 0), 1e-6);
    EXPECT_NEAR(limits.joint_limits(i, 1), target_limits.joint_limits(i, 1), 1e-6);
    EXPECT_NEAR(limits.velocity_limits(i), target_limits.velocity_limits(i), 1e-6);
    EXPECT_NEAR(limits.acceleration_limits(i), target_limits.acceleration_limits(i), 1e-6);
  }
}

/**
 * @brief Run kinematics setJointLimits function test
 * @param kin Kinematic object to test
 */
inline void runKinSetJointLimitsTest(tesseract_kinematics::ForwardKinematics& kin)
{
  //////////////////////////////////////////////////////////////////
  // Test setting forward kinematics joint limits
  //////////////////////////////////////////////////////////////////
  tesseract_common::KinematicLimits limits = kin.getLimits();
  EXPECT_TRUE(limits.joint_limits.rows() > 0);
  EXPECT_TRUE(limits.velocity_limits.rows() > 0);
  EXPECT_TRUE(limits.acceleration_limits.rows() > 0);

  // Check limits
  for (Eigen::Index i = 0; i < limits.joint_limits.rows(); ++i)
  {
    limits.joint_limits(i, 0) = -5.0 - double(i);
    limits.joint_limits(i, 1) = 5.0 + double(i);
    limits.velocity_limits(i) = 10.0 + double(i);
    limits.acceleration_limits(i) = 5.0 + double(i);
  }

  kin.setLimits(limits);
  runKinJointLimitsTest(kin.getLimits(), limits);

  // Test failure
  tesseract_common::KinematicLimits limits_empty;
  EXPECT_ANY_THROW(kin.setLimits(limits_empty));  // NOLINT
}

/**
 * @brief Run inverse kinematics setJointLimits function test
 * @param kin Inverse kinematic object to test
 */
inline void runKinSetJointLimitsTest(tesseract_kinematics::InverseKinematics& kin)
{
  //////////////////////////////////////////////////////////////////
  // Test setting forward kinematics joint limits
  //////////////////////////////////////////////////////////////////
  tesseract_common::KinematicLimits limits = kin.getLimits();
  EXPECT_TRUE(limits.joint_limits.rows() > 0);
  EXPECT_TRUE(limits.velocity_limits.rows() > 0);
  EXPECT_TRUE(limits.acceleration_limits.rows() > 0);

  // Check limits
  for (Eigen::Index i = 0; i < limits.joint_limits.rows(); ++i)
  {
    limits.joint_limits(i, 0) = -5.0 - double(i);
    limits.joint_limits(i, 1) = 5.0 + double(i);
    limits.velocity_limits(i) = 10.0 + double(i);
    limits.acceleration_limits(i) = 5.0 + double(i);
  }

  kin.setLimits(limits);
  runKinJointLimitsTest(kin.getLimits(), limits);
}

/**
 * @brief Check if two vectors of strings are equal but ignore order
 * @param names Vector to check
 * @param target_names Target to compare against
 */
inline void runStringVectorEqualTest(std::vector<std::string> names, std::vector<std::string> target_names)
{
  EXPECT_EQ(names.size(), target_names.size());
  EXPECT_FALSE(names.empty());
  EXPECT_FALSE(target_names.empty());
  EXPECT_TRUE(tesseract_common::isIdentical(names, target_names, false));
}

/**
 * @brief Run inverse kinematics test comparing the inverse solution to the forward solution
 * @param inv_kin The inverse kinematics object
 * @param fwd_kin The forward kinematics object to compare to
 * @param target_pose The target pose to solve inverse kinematics for
 * @param seed The seed used for solving inverse kinematics
 */
inline void runInvKinTest(const tesseract_kinematics::InverseKinematics& inv_kin,
                          const tesseract_kinematics::ForwardKinematics& fwd_kin,
                          const Eigen::Isometry3d& target_pose,
                          const Eigen::VectorXd& seed)
{
  ///////////////////////////
  // Test Inverse kinematics
  ///////////////////////////
  IKSolutions solutions = inv_kin.calcInvKin(target_pose, seed);
  EXPECT_TRUE(!solutions.empty());

  for (const auto& sol : solutions)
  {
    Eigen::Isometry3d result = fwd_kin.calcFwdKin(sol);
    EXPECT_TRUE(target_pose.translation().isApprox(result.translation(), 1e-4));

    Eigen::Quaterniond rot_pose(target_pose.rotation());
    Eigen::Quaterniond rot_result(result.rotation());
    EXPECT_TRUE(rot_pose.isApprox(rot_result, 1e-3));
  }
}

inline void runFwdKinIIWATest(tesseract_kinematics::ForwardKinematics& kin)
{
  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////

  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Zero(8)));
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Constant(7, std::numeric_limits<double>::max())));
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Constant(7, -std::numeric_limits<double>::max())));
  EXPECT_TRUE(kin.checkJoints(Eigen::VectorXd::Zero(7)));

  Eigen::Isometry3d pose;
  Eigen::VectorXd jvals;
  jvals.resize(7);
  jvals.setZero();

  pose = kin.calcFwdKin(jvals, "base_link");
  EXPECT_TRUE(pose.isApprox(Eigen::Isometry3d::Identity()));

  ///////////////////////////
  // Test forward kinematics
  ///////////////////////////
  {
    pose = kin.calcFwdKin(jvals, "link_1");
    Eigen::Isometry3d result;
    result.setIdentity();
    result.translation()[0] = 0;
    result.translation()[1] = 0;
    result.translation()[2] = 0;
    EXPECT_TRUE(pose.isApprox(result));
  }

  {
    pose = kin.calcFwdKin(jvals, "link_2");
    Eigen::Isometry3d result;
    result.setIdentity();
    result.translation()[0] = -0.00043624;
    result.translation()[1] = 0;
    result.translation()[2] = 0.36;
    EXPECT_TRUE(pose.isApprox(result));
  }

  {
    pose = kin.calcFwdKin(jvals, "link_3");
    Eigen::Isometry3d result;
    result.setIdentity();
    result.translation()[0] = -0.00043624;
    result.translation()[1] = 0;
    result.translation()[2] = 0.36;
    EXPECT_TRUE(pose.isApprox(result));
  }

  {
    pose = kin.calcFwdKin(jvals, "link_4");
    Eigen::Isometry3d result;
    result.setIdentity();
    result.translation()[0] = 0;
    result.translation()[1] = 0;
    result.translation()[2] = 0.36 + 0.42;
    EXPECT_TRUE(pose.isApprox(result));
  }

  {
    pose = kin.calcFwdKin(jvals, "link_5");
    Eigen::Isometry3d result;
    result.setIdentity();
    result.translation()[0] = 0;
    result.translation()[1] = 0;
    result.translation()[2] = 0.36 + 0.42;
    EXPECT_TRUE(pose.isApprox(result));
  }

  {
    pose = kin.calcFwdKin(jvals, "link_6");
    Eigen::Isometry3d result;
    result.setIdentity();
    result.translation()[0] = 0;
    result.translation()[1] = 0;
    result.translation()[2] = 0.36 + 0.42 + 0.4;
    EXPECT_TRUE(pose.isApprox(result));
  }

  {
    pose = kin.calcFwdKin(jvals, "link_7");
    Eigen::Isometry3d result;
    result.setIdentity();
    result.translation()[0] = 0;
    result.translation()[1] = 0;
    result.translation()[2] = 0.36 + 0.42 + 0.4;
    EXPECT_TRUE(pose.isApprox(result));
  }

  pose = kin.calcFwdKin(jvals, "tool0");
  Eigen::Isometry3d result;
  result.setIdentity();
  result.translation()[0] = 0;
  result.translation()[1] = 0;
  result.translation()[2] = 1.306;
  EXPECT_TRUE(pose.isApprox(result));
}

inline void runFwdKinAllPosesIIWATest(tesseract_kinematics::ForwardKinematics& kin, bool supported = true)
{
  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  tesseract_common::VectorIsometry3d poses;
  Eigen::VectorXd jvals;
  jvals.resize(7);
  jvals.setZero();

  ///////////////////////////
  // Test forward kinematics
  ///////////////////////////
  if (!supported)
  {
    EXPECT_ANY_THROW(kin.calcFwdKinAll(jvals));  // NOLINT
  }
  else
  {
    poses = kin.calcFwdKinAll(jvals);
    Eigen::Isometry3d result;
    result.setIdentity();
    result.translation()[0] = 0;
    result.translation()[1] = 0;
    result.translation()[2] = 0;
    EXPECT_TRUE(poses[0].isApprox(result));

    result.setIdentity();
    result.translation()[0] = -0.00043624;
    result.translation()[1] = 0;
    result.translation()[2] = 0.36;
    EXPECT_TRUE(poses[1].isApprox(result));

    result.setIdentity();
    result.translation()[0] = -0.00043624;
    result.translation()[1] = 0;
    result.translation()[2] = 0.36;
    EXPECT_TRUE(poses[2].isApprox(result));

    result.setIdentity();
    result.translation()[0] = 0;
    result.translation()[1] = 0;
    result.translation()[2] = 0.36 + 0.42;
    EXPECT_TRUE(poses[3].isApprox(result));

    result.setIdentity();
    result.translation()[0] = 0;
    result.translation()[1] = 0;
    result.translation()[2] = 0.36 + 0.42;
    EXPECT_TRUE(poses[4].isApprox(result));

    result.setIdentity();
    result.translation()[0] = 0;
    result.translation()[1] = 0;
    result.translation()[2] = 0.36 + 0.42 + 0.4;
    EXPECT_TRUE(poses[5].isApprox(result));

    result.setIdentity();
    result.translation()[0] = 0;
    result.translation()[1] = 0;
    result.translation()[2] = 0.36 + 0.42 + 0.4;
    EXPECT_TRUE(poses[6].isApprox(result));

    result.setIdentity();
    result.translation()[0] = 0;
    result.translation()[1] = 0;
    result.translation()[2] = 1.306;
    EXPECT_TRUE(poses[7].isApprox(result));
  }
}

inline void runJacobianIIWATest(tesseract_kinematics::ForwardKinematics& kin, bool is_kin_tree = false)
{
  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::MatrixXd jacobian, numerical_jacobian;
  Eigen::VectorXd jvals;
  jvals.resize(7);

  jvals(0) = -0.785398;
  jvals(1) = 0.785398;
  jvals(2) = -0.785398;
  jvals(3) = 0.785398;
  jvals(4) = -0.785398;
  jvals(5) = 0.785398;
  jvals(6) = -0.785398;

  ///////////////////////////
  // Test Jacobian
  ///////////////////////////
  Eigen::Vector3d link_point(0, 0, 0);
  runJacobianTest(kin, jvals, "base_link", link_point, Eigen::Isometry3d::Identity());
  runJacobianTest(kin, jvals, "link_1", link_point, Eigen::Isometry3d::Identity());
  runJacobianTest(kin, jvals, "link_2", link_point, Eigen::Isometry3d::Identity());
  runJacobianTest(kin, jvals, "link_3", link_point, Eigen::Isometry3d::Identity());
  runJacobianTest(kin, jvals, "link_4", link_point, Eigen::Isometry3d::Identity());
  runJacobianTest(kin, jvals, "link_5", link_point, Eigen::Isometry3d::Identity());
  runJacobianTest(kin, jvals, "link_6", link_point, Eigen::Isometry3d::Identity());
  runJacobianTest(kin, jvals, "link_7", link_point, Eigen::Isometry3d::Identity());
  runJacobianTest(kin, jvals, "tool0", link_point, Eigen::Isometry3d::Identity());
  if (!is_kin_tree)
  {
    runJacobianTest(kin, jvals, "", link_point, Eigen::Isometry3d::Identity());
  }
  else
  {
    EXPECT_ANY_THROW(runJacobianTest(kin, jvals, "", link_point, Eigen::Isometry3d::Identity()));  // NOLINT
  }

  ///////////////////////////
  // Test Jacobian at Point
  ///////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    runJacobianTest(kin, jvals, "base_link", link_point, Eigen::Isometry3d::Identity());
    runJacobianTest(kin, jvals, "link_1", link_point, Eigen::Isometry3d::Identity());
    runJacobianTest(kin, jvals, "link_2", link_point, Eigen::Isometry3d::Identity());
    runJacobianTest(kin, jvals, "link_3", link_point, Eigen::Isometry3d::Identity());
    runJacobianTest(kin, jvals, "link_4", link_point, Eigen::Isometry3d::Identity());
    runJacobianTest(kin, jvals, "link_5", link_point, Eigen::Isometry3d::Identity());
    runJacobianTest(kin, jvals, "link_6", link_point, Eigen::Isometry3d::Identity());
    runJacobianTest(kin, jvals, "link_7", link_point, Eigen::Isometry3d::Identity());
    runJacobianTest(kin, jvals, "tool0", link_point, Eigen::Isometry3d::Identity());
    if (!is_kin_tree)
    {
      runJacobianTest(kin, jvals, "", link_point, Eigen::Isometry3d::Identity());
    }
    else
    {
      EXPECT_ANY_THROW(runJacobianTest(kin, jvals, "", link_point, Eigen::Isometry3d::Identity()));  // NOLINT
    }
  }

  ///////////////////////////////////////////
  // Test Jacobian with change base
  ///////////////////////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    link_point = Eigen::Vector3d(0, 0, 0);
    Eigen::Isometry3d change_base;
    change_base.setIdentity();
    change_base(0, 0) = 0;
    change_base(1, 0) = 1;
    change_base(0, 1) = -1;
    change_base(1, 1) = 0;
    change_base.translation() = Eigen::Vector3d(0, 0, 0);
    change_base.translation()[k] = 1;

    runJacobianTest(kin, jvals, "base_link", link_point, change_base);
    runJacobianTest(kin, jvals, "link_1", link_point, change_base);
    runJacobianTest(kin, jvals, "link_2", link_point, change_base);
    runJacobianTest(kin, jvals, "link_3", link_point, change_base);
    runJacobianTest(kin, jvals, "link_4", link_point, change_base);
    runJacobianTest(kin, jvals, "link_5", link_point, change_base);
    runJacobianTest(kin, jvals, "link_6", link_point, change_base);
    runJacobianTest(kin, jvals, "link_7", link_point, change_base);
    runJacobianTest(kin, jvals, "tool0", link_point, change_base);
    if (!is_kin_tree)
    {
      runJacobianTest(kin, jvals, "", link_point, change_base);
    }
    else
    {
      EXPECT_ANY_THROW(runJacobianTest(kin, jvals, "", link_point, change_base));  // NOLINT
    }
  }

  ///////////////////////////////////////////
  // Test Jacobian at point with change base
  ///////////////////////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    Eigen::Isometry3d change_base;
    change_base.setIdentity();
    change_base(0, 0) = 0;
    change_base(1, 0) = 1;
    change_base(0, 1) = -1;
    change_base(1, 1) = 0;
    change_base.translation() = link_point;

    runJacobianTest(kin, jvals, "base_link", link_point, change_base);
    runJacobianTest(kin, jvals, "link_1", link_point, change_base);
    runJacobianTest(kin, jvals, "link_2", link_point, change_base);
    runJacobianTest(kin, jvals, "link_3", link_point, change_base);
    runJacobianTest(kin, jvals, "link_4", link_point, change_base);
    runJacobianTest(kin, jvals, "link_5", link_point, change_base);
    runJacobianTest(kin, jvals, "link_6", link_point, change_base);
    runJacobianTest(kin, jvals, "link_7", link_point, change_base);
    runJacobianTest(kin, jvals, "tool0", link_point, change_base);
    if (!is_kin_tree)
    {
      runJacobianTest(kin, jvals, "", link_point, change_base);
    }
    else
    {
      EXPECT_ANY_THROW(runJacobianTest(kin, jvals, "", link_point, change_base));  // NOLINT
    }
  }
}

inline void runActiveLinkNamesIIWATest(const tesseract_kinematics::ForwardKinematics& kin, bool isKinTree)
{
  std::vector<std::string> target_active_link_names = { "link_1", "link_2", "link_3", "link_4",
                                                        "link_5", "link_6", "link_7", "tool0" };

  std::vector<std::string> target_link_names = target_active_link_names;
  target_link_names.emplace_back("base_link");

  std::vector<std::string> target_tree_link_names = target_link_names;
  target_tree_link_names.emplace_back("base");

  std::vector<std::string> link_names = kin.getActiveLinkNames();
  runStringVectorEqualTest(link_names, target_active_link_names);

  if (!isKinTree)
  {
    link_names = kin.getLinkNames();
    runStringVectorEqualTest(link_names, target_link_names);
  }
  else
  {
    link_names = kin.getLinkNames();
    runStringVectorEqualTest(link_names, target_tree_link_names);
  }
}

inline void runActiveLinkNamesIIWATest(const tesseract_kinematics::InverseKinematics& kin)
{
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Zero(8)));
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Constant(7, std::numeric_limits<double>::max())));
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Constant(7, -std::numeric_limits<double>::max())));
  EXPECT_TRUE(kin.checkJoints(Eigen::VectorXd::Zero(7)));

  std::vector<std::string> target_active_link_names = { "link_1", "link_2", "link_3", "link_4",
                                                        "link_5", "link_6", "link_7", "tool0" };
  std::vector<std::string> target_link_names = target_active_link_names;
  target_link_names.emplace_back("base_link");

  std::vector<std::string> link_names = kin.getActiveLinkNames();
  runStringVectorEqualTest(link_names, target_active_link_names);

  link_names = kin.getLinkNames();
  runStringVectorEqualTest(link_names, target_link_names);
}

inline void runActiveLinkNamesABBTest(const tesseract_kinematics::InverseKinematics& kin)
{
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Zero(7)));
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Constant(6, std::numeric_limits<double>::max())));
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Constant(6, -std::numeric_limits<double>::max())));
  EXPECT_TRUE(kin.checkJoints(Eigen::VectorXd::Zero(6)));

  std::vector<std::string> target_active_link_names = { "link_1", "link_2", "link_3", "link_4",
                                                        "link_5", "link_6", "tool0" };
  std::vector<std::string> target_link_names = target_active_link_names;
  target_link_names.emplace_back("base_link");

  std::vector<std::string> link_names = kin.getActiveLinkNames();
  runStringVectorEqualTest(link_names, target_active_link_names);

  link_names = kin.getLinkNames();
  runStringVectorEqualTest(link_names, target_link_names);
}

inline void runActiveLinkNamesURTest(const tesseract_kinematics::InverseKinematics& kin)
{
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Zero(7)));
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Constant(6, std::numeric_limits<double>::max())));
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Constant(6, -std::numeric_limits<double>::max())));
  EXPECT_TRUE(kin.checkJoints(Eigen::VectorXd::Zero(6)));

  std::vector<std::string> target_active_link_names = { "shoulder_link", "upper_arm_link", "forearm_link",
                                                        "wrist_1_link",  "wrist_2_link",   "wrist_3_link",
                                                        "tool0" };
  std::vector<std::string> target_link_names = target_active_link_names;
  target_link_names.emplace_back("base_link");

  std::vector<std::string> link_names = kin.getActiveLinkNames();
  runStringVectorEqualTest(link_names, target_active_link_names);

  link_names = kin.getLinkNames();
  runStringVectorEqualTest(link_names, target_link_names);
}

inline void runActiveLinkNamesABBOnPositionerTest(const tesseract_kinematics::InverseKinematics& kin)
{
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Zero(8)));
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Constant(7, std::numeric_limits<double>::max())));
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Constant(7, -std::numeric_limits<double>::max())));
  EXPECT_TRUE(kin.checkJoints(Eigen::VectorXd::Zero(7)));

  std::vector<std::string> target_active_link_names = { "positioner_tool0", "base_link", "link_1", "link_2", "link_3",
                                                        "link_4",           "link_5",    "link_6", "tool0" };
  std::vector<std::string> target_link_names = target_active_link_names;
  target_link_names.emplace_back("positioner_base_link");

  std::vector<std::string> link_names = kin.getActiveLinkNames();
  runStringVectorEqualTest(link_names, target_active_link_names);

  link_names = kin.getLinkNames();
  runStringVectorEqualTest(link_names, target_link_names);
}

inline void runActiveLinkNamesABBExternalPositionerTest(const tesseract_kinematics::InverseKinematics& kin)
{
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Zero(8)));
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Constant(7, std::numeric_limits<double>::max())));
  EXPECT_FALSE(kin.checkJoints(Eigen::VectorXd::Constant(7, -std::numeric_limits<double>::max())));
  EXPECT_TRUE(kin.checkJoints(Eigen::VectorXd::Zero(7)));

  std::vector<std::string> target_active_link_names = { "positioner_tool0", "link_1", "link_2", "link_3",
                                                        "link_4",           "link_5", "link_6", "tool0" };
  std::vector<std::string> target_link_names = target_active_link_names;
  target_link_names.emplace_back("world");
  target_link_names.emplace_back("base_link");
  target_link_names.emplace_back("positioner_base_link");

  std::vector<std::string> link_names = kin.getActiveLinkNames();
  runStringVectorEqualTest(link_names, target_active_link_names);

  link_names = kin.getLinkNames();
  runStringVectorEqualTest(link_names, target_link_names);
}

inline void runInvKinIIWATest(const tesseract_kinematics::InverseKinematicsFactory& inv_kin_factory,
                              const tesseract_kinematics::ForwardKinematicsFactory& fwd_kin_factory,
                              const std::string& solver_name,
                              tesseract_kinematics::InverseKinematicsFactoryType factory_type)
{
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraphIIWA();

  // Inverse target pose and seed
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0;
  pose.translation()[1] = 0;
  pose.translation()[2] = 1.306;

  Eigen::VectorXd seed;
  seed.resize(7);
  seed(0) = -0.785398;
  seed(1) = 0.785398;
  seed(2) = -0.785398;
  seed(3) = 0.785398;
  seed(4) = -0.785398;
  seed(5) = 0.785398;
  seed(6) = -0.785398;

  // Check create method with empty scene graph
  auto scene_graph_empty = std::make_shared<tesseract_scene_graph::SceneGraph>();
  tesseract_kinematics::InverseKinematics::Ptr kin_empty =
      inv_kin_factory.create(scene_graph_empty, "base_link", "tool0", "manip");
  EXPECT_TRUE(kin_empty == nullptr);

  // Check create method using base_link and tool0
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin =
      fwd_kin_factory.create(scene_graph, "base_link", "tool0", "manip");
  EXPECT_TRUE(fwd_kin != nullptr);

  EXPECT_EQ(inv_kin_factory.getName(), solver_name);
  EXPECT_EQ(inv_kin_factory.getType(), factory_type);

  tesseract_kinematics::InverseKinematics::Ptr inv_kin =
      inv_kin_factory.create(scene_graph, "base_link", "tool0", "manip");
  EXPECT_TRUE(inv_kin != nullptr);
  EXPECT_EQ(inv_kin->getName(), "manip");
  EXPECT_EQ(inv_kin->getSolverName(), solver_name);
  EXPECT_EQ(inv_kin->numJoints(), 7);
  EXPECT_EQ(inv_kin->getBaseLinkName(), "base_link");
  EXPECT_EQ(inv_kin->getTipLinkName(), "tool0");
  tesseract_common::KinematicLimits target_limits = getTargetLimits(scene_graph, inv_kin->getJointNames());

  runInvKinTest(*inv_kin, *fwd_kin, pose, seed);
  runActiveLinkNamesIIWATest(*inv_kin);
  runKinJointLimitsTest(inv_kin->getLimits(), target_limits);

  // Check create method using chain pairs
  tesseract_kinematics::InverseKinematics::Ptr inv_kin2 =
      inv_kin_factory.create(scene_graph, { std::make_pair("base_link", "tool0") }, "manip");
  EXPECT_TRUE(inv_kin2 != nullptr);
  EXPECT_EQ(inv_kin2->getName(), "manip");
  EXPECT_EQ(inv_kin2->getSolverName(), solver_name);
  EXPECT_EQ(inv_kin2->numJoints(), 7);
  EXPECT_EQ(inv_kin2->getBaseLinkName(), "base_link");
  EXPECT_EQ(inv_kin2->getTipLinkName(), "tool0");

  runInvKinTest(*inv_kin2, *fwd_kin, pose, seed);
  runActiveLinkNamesIIWATest(*inv_kin2);
  runKinJointLimitsTest(inv_kin2->getLimits(), target_limits);

  // Check cloned
  tesseract_kinematics::InverseKinematics::Ptr inv_kin3 = inv_kin->clone();
  EXPECT_TRUE(inv_kin3 != nullptr);
  EXPECT_EQ(inv_kin3->getName(), "manip");
  EXPECT_EQ(inv_kin3->getSolverName(), solver_name);
  EXPECT_EQ(inv_kin3->numJoints(), 7);
  EXPECT_EQ(inv_kin3->getBaseLinkName(), "base_link");
  EXPECT_EQ(inv_kin3->getTipLinkName(), "tool0");

  runInvKinTest(*inv_kin3, *fwd_kin, pose, seed);
  runActiveLinkNamesIIWATest(*inv_kin3);
  runKinJointLimitsTest(inv_kin3->getLimits(), target_limits);

  // Check update
  inv_kin3->update();
  EXPECT_TRUE(inv_kin3 != nullptr);
  EXPECT_EQ(inv_kin3->getName(), "manip");
  EXPECT_EQ(inv_kin3->getSolverName(), solver_name);
  EXPECT_EQ(inv_kin3->numJoints(), 7);
  EXPECT_EQ(inv_kin3->getBaseLinkName(), "base_link");
  EXPECT_EQ(inv_kin3->getTipLinkName(), "tool0");

  runInvKinTest(*inv_kin3, *fwd_kin, pose, seed);
  runActiveLinkNamesIIWATest(*inv_kin3);
  runKinJointLimitsTest(inv_kin3->getLimits(), target_limits);

  // Test setJointLimits
  runKinSetJointLimitsTest(*inv_kin);

  // Test failure
  inv_kin = inv_kin_factory.create(scene_graph, "missing_link", "tool0", "manip");
  EXPECT_TRUE(inv_kin == nullptr);

  inv_kin2 = inv_kin_factory.create(scene_graph, { std::make_pair("missing_link", "tool0") }, "manip");
  EXPECT_TRUE(inv_kin2 == nullptr);

  inv_kin = inv_kin_factory.create(nullptr, "base_link", "tool0", "manip");
  EXPECT_TRUE(inv_kin == nullptr);

  inv_kin2 = inv_kin_factory.create(nullptr, { std::make_pair("base_link", "tool0") }, "manip");
  EXPECT_TRUE(inv_kin2 == nullptr);
}

}  // namespace test_suite
}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_KIN_TEST_SUITE_H
