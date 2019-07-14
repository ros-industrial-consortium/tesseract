#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
#include <tesseract_scene_graph/parser/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_kinematics/kdl/kdl_fwd_kin_chain.h"
#include "tesseract_kinematics/kdl/kdl_fwd_kin_tree.h"
#include "tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h"
#include "tesseract_kinematics/kdl/kdl_inv_kin_chain_nr.h"
#include "tesseract_kinematics/core/utils.h"

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find("/");
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

tesseract_scene_graph::SceneGraph::Ptr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_scene_graph::ResourceLocatorFn locator = locateResource;
  return tesseract_scene_graph::parseURDFFile(path, locator);
}

void runFwdKinTest(tesseract_kinematics::ForwardKinematics& kin)
{
  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::Isometry3d pose;
  Eigen::VectorXd jvals;
  jvals.resize(7);
  jvals.setZero();

  EXPECT_TRUE(kin.calcFwdKin(pose, jvals, "base_link"));
  EXPECT_TRUE(pose.isApprox(Eigen::Isometry3d::Identity()));

  ///////////////////////////
  // Test forward kinematics
  ///////////////////////////
  pose.setIdentity();
  EXPECT_TRUE(kin.calcFwdKin(pose, jvals, "tool0"));
  Eigen::Isometry3d result;
  result.setIdentity();
  result.translation()[0] = 0;
  result.translation()[1] = 0;
  result.translation()[2] = 1.306;
  EXPECT_TRUE(pose.isApprox(result));
}

void runJacobianTest(tesseract_kinematics::ForwardKinematics& kin)
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
  jacobian.resize(6, 7);
  EXPECT_TRUE(kin.calcJacobian(jacobian, jvals, "tool0"));

  Eigen::Vector3d link_point(0, 0, 0);
  numerical_jacobian.resize(6, 7);
  tesseract_kinematics::numericalJacobian(
      numerical_jacobian, Eigen::Isometry3d::Identity(), kin, jvals, "tool0", link_point);

  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 7; ++j)
      EXPECT_NEAR(numerical_jacobian(i, j), jacobian(i, j), 1e-3);

  ///////////////////////////
  // Test Jacobian at Point
  ///////////////////////////
  Eigen::Isometry3d pose;
  kin.calcFwdKin(pose, jvals, "tool0");
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    // calcJacobian requires the link point to be in the base frame for which the jacobian is calculated.
    EXPECT_TRUE(kin.calcJacobian(jacobian, jvals, "tool0"));
    tesseract_kinematics::jacobianChangeRefPoint(jacobian, pose.linear() * link_point);

    numerical_jacobian.resize(6, 7);
    tesseract_kinematics::numericalJacobian(
        numerical_jacobian, Eigen::Isometry3d::Identity(), kin, jvals, "tool0", link_point);

    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 7; ++j)
        EXPECT_NEAR(numerical_jacobian(i, j), jacobian(i, j), 1e-3);
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

    EXPECT_TRUE(kin.calcJacobian(jacobian, jvals, "tool0"));
    tesseract_kinematics::jacobianChangeBase(jacobian, change_base);

    numerical_jacobian.resize(6, 7);
    tesseract_kinematics::numericalJacobian(numerical_jacobian, change_base, kin, jvals, "tool0", link_point);

    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 7; ++j)
        EXPECT_NEAR(numerical_jacobian(i, j), jacobian(i, j), 1e-3);
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

    kin.calcFwdKin(pose, jvals, "tool0");
    // calcJacobian requires the link point to be in the base frame for which the jacobian is calculated.
    EXPECT_TRUE(kin.calcJacobian(jacobian, jvals, "tool0"));
    tesseract_kinematics::jacobianChangeBase(jacobian, change_base);
    tesseract_kinematics::jacobianChangeRefPoint(jacobian, (change_base * pose).linear() * link_point);

    numerical_jacobian.resize(6, 7);
    tesseract_kinematics::numericalJacobian(numerical_jacobian, change_base, kin, jvals, "tool0", link_point);

    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 7; ++j)
        EXPECT_NEAR(numerical_jacobian(i, j), jacobian(i, j), 1e-3);
  }
}

void runActiveLinkNamesTest(tesseract_kinematics::ForwardKinematics& kin, bool isKinTree)
{
  std::vector<std::string> link_names = kin.getActiveLinkNames();
  EXPECT_TRUE(link_names.size() == 8);
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "base_link") == link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_1") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_2") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_3") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_4") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_5") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_6") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_7") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "tool0") != link_names.end());

  if (!isKinTree)
  {
    link_names = kin.getLinkNames();
    EXPECT_TRUE(link_names.size() == 9);
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "base_link") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_1") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_2") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_3") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_4") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_5") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_6") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_7") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "tool0") != link_names.end());
  }
  else
  {
    link_names = kin.getLinkNames();
    EXPECT_TRUE(link_names.size() == 10);
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "base_link") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "base") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_1") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_2") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_3") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_4") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_5") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_6") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_7") != link_names.end());
    EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "tool0") != link_names.end());
  }
}

void runInvKinTest(const tesseract_kinematics::InverseKinematics& inv_kin,
                   const tesseract_kinematics::ForwardKinematics& fwd_kin)
{
  //////////////////////////////////////////////////////////////////
  // Test inverse kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
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

  ///////////////////////////
  // Test Inverse kinematics
  ///////////////////////////
  Eigen::VectorXd solutions;
  EXPECT_TRUE(inv_kin.calcInvKin(solutions, pose, seed));

  Eigen::Isometry3d result;
  EXPECT_TRUE(fwd_kin.calcFwdKin(result, solutions));
  EXPECT_TRUE(pose.translation().isApprox(result.translation(), 1e-4));

  Eigen::Quaterniond rot_pose(pose.rotation());
  Eigen::Quaterniond rot_result(result.rotation());
  EXPECT_TRUE(rot_pose.isApprox(rot_result, 1e-3));
}

TEST(TesseractKinematicsUnit, KDLKinChainActiveLinkNamesUnit)
{
  tesseract_kinematics::KDLFwdKinChain kin;
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  EXPECT_TRUE(kin.init(scene_graph, "base_link", "tool0", "manip"));

  runActiveLinkNamesTest(kin, false);
}

TEST(TesseractKinematicsUnit, KDLKinTreeActiveLinkNamesUnit)
{
  tesseract_kinematics::KDLFwdKinTree kin;
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  std::vector<std::string> joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                           "joint_a5", "joint_a6", "joint_a7" };

  std::unordered_map<std::string, double> start_state;
  start_state["joint_a1"] = 0;
  start_state["joint_a2"] = 0;
  start_state["joint_a3"] = 0;
  start_state["joint_a4"] = 0;
  start_state["joint_a5"] = 0;
  start_state["joint_a6"] = 0;
  start_state["joint_a7"] = 0;

  EXPECT_TRUE(kin.init(scene_graph, joint_names, "manip", start_state));

  runActiveLinkNamesTest(kin, true);
}

TEST(TesseractKinematicsUnit, KDLKinChainForwardKinematicUnit)
{
  tesseract_kinematics::KDLFwdKinChain kin;
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  EXPECT_TRUE(kin.init(scene_graph, "base_link", "tool0", "manip"));

  runFwdKinTest(kin);
}

TEST(TesseractKinematicsUnit, KDLKinTreeForwardKinematicUnit)
{
  tesseract_kinematics::KDLFwdKinTree kin;
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  std::vector<std::string> joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                           "joint_a5", "joint_a6", "joint_a7" };

  std::unordered_map<std::string, double> start_state;
  start_state["joint_a1"] = 0;
  start_state["joint_a2"] = 0;
  start_state["joint_a3"] = 0;
  start_state["joint_a4"] = 0;
  start_state["joint_a5"] = 0;
  start_state["joint_a6"] = 0;
  start_state["joint_a7"] = 0;

  EXPECT_TRUE(kin.init(scene_graph, joint_names, "manip", start_state));

  runFwdKinTest(kin);
}

TEST(TesseractKinematicsUnit, KDLKinChainJacobianUnit)
{
  tesseract_kinematics::KDLFwdKinChain kin;
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  EXPECT_TRUE(kin.init(scene_graph, "base_link", "tool0", "manip"));

  runJacobianTest(kin);
}

TEST(TesseractKinematicsUnit, KDLKinTreeJacobianUnit)
{
  tesseract_kinematics::KDLFwdKinTree kin;
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  std::vector<std::string> joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                           "joint_a5", "joint_a6", "joint_a7" };

  std::unordered_map<std::string, double> start_state;
  start_state["joint_a1"] = 0;
  start_state["joint_a2"] = 0;
  start_state["joint_a3"] = 0;
  start_state["joint_a4"] = 0;
  start_state["joint_a5"] = 0;
  start_state["joint_a6"] = 0;
  start_state["joint_a7"] = 0;

  EXPECT_TRUE(kin.init(scene_graph, joint_names, "manip", start_state));

  runJacobianTest(kin);
}

TEST(TesseractKinematicsUnit, KDLKinChainLMAInverseKinematicUnit)
{
  tesseract_kinematics::KDLInvKinChainLMA inv_kin;
  tesseract_kinematics::KDLFwdKinChain fwd_kin;
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  EXPECT_TRUE(inv_kin.init(scene_graph, "base_link", "tool0", "manip"));
  EXPECT_TRUE(fwd_kin.init(scene_graph, "base_link", "tool0", "manip"));

  runInvKinTest(inv_kin, fwd_kin);
}

TEST(TesseractKinematicsUnit, KDLKinChainNRInverseKinematicUnit)
{
  tesseract_kinematics::KDLInvKinChainNR inv_kin;
  tesseract_kinematics::KDLFwdKinChain fwd_kin;
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  EXPECT_TRUE(inv_kin.init(scene_graph, "base_link", "tool0", "manip"));
  EXPECT_TRUE(fwd_kin.init(scene_graph, "base_link", "tool0", "manip"));

  runInvKinTest(inv_kin, fwd_kin);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
