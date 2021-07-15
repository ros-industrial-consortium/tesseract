#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <type_traits>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_common/sfinae_utils.h>
#include <tesseract_common/resource.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/types.h>
#include <tesseract_common/any.h>
#include <tesseract_common/kinematic_limits.h>

TEST(TesseractCommonUnit, isNumeric)  // NOLINT
{
  std::vector<std::string> true_test = { "1",     "1.5",  "-1",     "-1.5",  "1e-5",    "1e5",
                                         "-1e-5", "-1e5", "1.0e-5", "1.0e5", "-1.0e-5", "-1.0e5" };

  EXPECT_TRUE(tesseract_common::isNumeric(true_test));
  for (const auto& s : true_test)
  {
    EXPECT_TRUE(tesseract_common::isNumeric(s));
  }

  std::vector<std::string> false_test = { "a", "test sdfs", "1 2", "1.0 2.0", "+", "-", "=" };
  EXPECT_FALSE(tesseract_common::isNumeric(false_test));
  for (const auto& s : false_test)
  {
    EXPECT_FALSE(tesseract_common::isNumeric(s));
  }

  std::string empty_string;
  EXPECT_FALSE(tesseract_common::isNumeric(empty_string));
}

TEST(TesseractCommonUnit, toNumeric)  // NOLINT
{
  std::vector<std::string> true_test = { "1",     "1.5",  "-1",     "-1.5",  "1e-5",    "1e5",
                                         "-1e-5", "-1e5", "1.0e-5", "1.0e5", "-1.0e-5", "-1.0e5" };

  std::vector<double> true_test_value = { 1, 1.5, -1, -1.5, 1e-5, 1e5, -1e-5, -1e5, 1.0e-5, 1.0e5, -1.0e-5, -1.0e5 };

  EXPECT_TRUE(tesseract_common::isNumeric(true_test));
  for (size_t i = 0; i < true_test.size(); ++i)
  {
    double value = 0;
    EXPECT_TRUE(tesseract_common::toNumeric<double>(true_test[i], value));
    EXPECT_NEAR(value, true_test_value[i], 1e-8);
  }

  std::vector<std::string> false_test = { "a", "test sdfs", "1 2", "1.0 2.0", "+", "-", "=" };
  EXPECT_FALSE(tesseract_common::isNumeric(false_test));
  for (const auto& s : false_test)
  {
    double value = 0;
    EXPECT_FALSE(tesseract_common::toNumeric(s, value));
    EXPECT_NEAR(value, 0, 1e-8);
  }

  std::string empty_string;
  double value = 0;
  EXPECT_FALSE(tesseract_common::toNumeric(empty_string, value));
}

TEST(TesseractCommonUnit, generateRandomNumber)  // NOLINT
{
  Eigen::MatrixX2d limits(4, 2);
  limits(0, 0) = -5;
  limits(0, 1) = 5;
  limits(1, 0) = 0;
  limits(1, 1) = 10;
  limits(2, 0) = 5;
  limits(2, 1) = 15;
  limits(3, 0) = -15;
  limits(3, 1) = -5;

  Eigen::VectorXd random_numbers = tesseract_common::generateRandomNumber(limits);
  EXPECT_EQ(limits.rows(), random_numbers.rows());
  for (long i = 0; i < limits.rows(); ++i)
  {
    EXPECT_LE(random_numbers(i), limits(i, 1));
    EXPECT_GE(random_numbers(i), limits(i, 0));
  }

  Eigen::MatrixX2d empty_limits;
  Eigen::VectorXd random_numbers2 = tesseract_common::generateRandomNumber(empty_limits);
  EXPECT_EQ(empty_limits.rows(), random_numbers2.rows());

  Eigen::MatrixX2d equal_limits(4, 2);
  equal_limits(0, 0) = 5;
  equal_limits(0, 1) = 5;
  equal_limits(1, 0) = 5;
  equal_limits(1, 1) = 5;
  equal_limits(2, 0) = 5;
  equal_limits(2, 1) = 5;
  equal_limits(3, 0) = 5;
  equal_limits(3, 1) = 5;
  Eigen::VectorXd random_numbers3 = tesseract_common::generateRandomNumber(equal_limits);
  EXPECT_EQ(equal_limits.rows(), random_numbers3.rows());
  for (long i = 0; i < equal_limits.rows(); ++i)
  {
    EXPECT_NEAR(random_numbers3(i), 5, 1e-5);
  }

  Eigen::MatrixX2d wrong_limits(4, 2);
  wrong_limits(0, 0) = 5;
  wrong_limits(0, 1) = -5;
  wrong_limits(1, 0) = 5;
  wrong_limits(1, 1) = -5;
  wrong_limits(2, 0) = 5;
  wrong_limits(2, 1) = -5;
  wrong_limits(3, 0) = 5;
  wrong_limits(3, 1) = -5;
  Eigen::VectorXd random_numbers4 = tesseract_common::generateRandomNumber(wrong_limits);
  EXPECT_EQ(wrong_limits.rows(), random_numbers4.rows());
  for (long i = 0; i < limits.rows(); ++i)
  {
    EXPECT_GE(random_numbers4(i), wrong_limits(i, 1));
    EXPECT_LE(random_numbers4(i), wrong_limits(i, 0));
  }
}

TEST(TesseractCommonUnit, trim)  // NOLINT
{
  std::string check1 = "    trim";
  std::string check2 = "trim    ";
  std::string check3 = "    trim    ";
  std::string check_trimmed = "trim";

  std::string s = check1;
  tesseract_common::rtrim(s);
  EXPECT_EQ(s, check1);
  tesseract_common::ltrim(s);
  EXPECT_EQ(s, check_trimmed);

  s = check2;
  tesseract_common::ltrim(s);
  EXPECT_EQ(s, check2);
  tesseract_common::rtrim(s);
  EXPECT_EQ(s, check_trimmed);

  s = check1;
  tesseract_common::trim(s);
  EXPECT_EQ(s, check_trimmed);

  s = check2;
  tesseract_common::trim(s);
  EXPECT_EQ(s, check_trimmed);

  s = check3;
  tesseract_common::trim(s);
  EXPECT_EQ(s, check_trimmed);
}

struct TestHasMemberFunction
{
  bool update() const { return true; }
  int add(int a) const { return a + 1; }
};

struct TestHasMemberWithArgFunction
{
  bool update(std::shared_ptr<TestHasMemberWithArgFunction>& p) { return (p == nullptr); }
  double add(double a, double b) const { return a + b; }
};

struct TestMissingMemberFunction
{
  bool missingUpdate() const { return false; }
  double add(int a) const { return a + 1; }
};

CREATE_MEMBER_CHECK(update);
CREATE_MEMBER_FUNC_INVOCABLE_CHECK(update, std::shared_ptr<T>&);
CREATE_MEMBER_FUNC_INVOCABLE_CHECK(add, double, double);
CREATE_MEMBER_FUNC_RETURN_TYPE_CHECK(add, int, int);
CREATE_MEMBER_FUNC_SIGNATURE_CHECK(add, double, double, double);

TEST(TesseractCommonUnit, sfinaeHasMemberFunction)  // NOLINT
{
  bool t_true = has_member_update<TestHasMemberFunction>::value;
  bool t_false = has_member_update<TestMissingMemberFunction>::value;
  EXPECT_TRUE(t_true);
  EXPECT_FALSE(t_false);
}

TEST(TesseractCommonUnit, sfinaeHasMemberFunctionInvocable)  // NOLINT
{
  bool i_update_true = has_member_func_invocable_update<TestHasMemberWithArgFunction>::value;
  bool i_add_true = has_member_func_invocable_add<TestHasMemberWithArgFunction>::value;
  bool i_update_false = has_member_func_invocable_update<TestHasMemberFunction>::value;
  bool i_add_false = has_member_func_invocable_add<TestHasMemberFunction>::value;
  EXPECT_TRUE(i_update_true);
  EXPECT_TRUE(i_add_true);
  EXPECT_FALSE(i_update_false);
  EXPECT_FALSE(i_add_false);
}

TEST(TesseractCommonUnit, sfinaeHasMemberFunctionWithReturnType)  // NOLINT
{
  bool i_add_true = has_member_func_return_type_add<TestHasMemberFunction>::value;
  bool t_add_false = has_member_func_return_type_add<TestMissingMemberFunction>::value;
  EXPECT_TRUE(i_add_true);
  EXPECT_FALSE(t_add_false);
}

TEST(TesseractCommonUnit, sfinaeHasMemberFunctionSignature)  // NOLINT
{
  bool i_add_true = has_member_func_signature_add<TestHasMemberWithArgFunction>::value;
  bool t_add_false = has_member_func_signature_add<TestMissingMemberFunction>::value;
  EXPECT_TRUE(i_add_true);
  EXPECT_FALSE(t_add_false);
}

TEST(TesseractCommonUnit, bytesResource)
{
  std::vector<uint8_t> data;
  for (uint8_t i = 0; i < 8; i++)
  {
    data.push_back(i);
  }

  std::shared_ptr<tesseract_common::BytesResource> bytes_resource =
      std::make_shared<tesseract_common::BytesResource>("package://test_package/data.bin", data);
  EXPECT_EQ(bytes_resource->getUrl(), "package://test_package/data.bin");
  EXPECT_EQ(bytes_resource->isFile(), false);
  EXPECT_EQ(bytes_resource->getFilePath(), "");
  EXPECT_EQ(bytes_resource->locateSubResource("test"), nullptr);
  auto data2 = bytes_resource->getResourceContents();
  ASSERT_EQ(data.size(), data2.size());
  for (size_t i = 0; i < data.size(); i++)
  {
    EXPECT_EQ(data[i], data2[i]);
  }
  auto data2_stream = bytes_resource->getResourceContentStream();
  for (size_t i = 0; i < data.size(); i++)
  {
    char data2_val;
    data2_stream->read(&data2_val, 1);
    EXPECT_EQ(data[i], *reinterpret_cast<uint8_t*>(&data2_val));
  }

  std::shared_ptr<tesseract_common::BytesResource> bytes_resource2 =
      std::make_shared<tesseract_common::BytesResource>("package://test_package/data.bin", &data[0], data.size());
  EXPECT_EQ(bytes_resource2->getUrl(), "package://test_package/data.bin");
  EXPECT_EQ(bytes_resource->getResourceContents().size(), data.size());
}

TEST(TesseractCommonUnit, ToolCenterPoint)
{
  {  // Empty tcp
    tesseract_common::ToolCenterPoint tcp;
    EXPECT_TRUE(tcp.empty());
    EXPECT_FALSE(tcp.isString());
    EXPECT_FALSE(tcp.isTransform());
    EXPECT_FALSE(tcp.isExternal());
    EXPECT_ANY_THROW(tcp.getString());
    EXPECT_ANY_THROW(tcp.getTransform());
    EXPECT_ANY_THROW(tcp.getExternalFrame());
  }

  {  // The tcp is a link attached to the tip of the kinematic chain
    tesseract_common::ToolCenterPoint tcp("tcp_link");
    EXPECT_FALSE(tcp.empty());
    EXPECT_TRUE(tcp.isString());
    EXPECT_FALSE(tcp.isTransform());
    EXPECT_FALSE(tcp.isExternal());
    EXPECT_EQ(tcp.getString(), "tcp_link");
    EXPECT_ANY_THROW(tcp.getTransform());
    EXPECT_ANY_THROW(tcp.getExternalFrame());
  }

  {  // The tcp is external
    tesseract_common::ToolCenterPoint tcp("external_tcp_link", true);
    EXPECT_FALSE(tcp.empty());
    EXPECT_TRUE(tcp.isString());
    EXPECT_FALSE(tcp.isTransform());
    EXPECT_TRUE(tcp.isExternal());
    EXPECT_EQ(tcp.getString(), "external_tcp_link");
    EXPECT_ANY_THROW(tcp.getTransform());
    EXPECT_ANY_THROW(tcp.getExternalFrame());

    tcp.setExternal(false);
    EXPECT_FALSE(tcp.isExternal());
    EXPECT_ANY_THROW(tcp.getExternalFrame());

    tcp.setExternal(true, "should_not_add");
    EXPECT_TRUE(tcp.isExternal());
    EXPECT_ANY_THROW(tcp.getExternalFrame());
  }

  {  // The tcp is external with transform
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0, 0, 0.25);
    tesseract_common::ToolCenterPoint tcp(pose, true, "external_frame");
    EXPECT_EQ(tcp.getExternalFrame(), "external_frame");
    EXPECT_TRUE(tcp.isExternal());
    EXPECT_TRUE(tcp.isTransform());
    EXPECT_TRUE(tcp.getTransform().isApprox(pose, 1e-6));

    // Set as external after construction
    tcp = tesseract_common::ToolCenterPoint(pose);
    tcp.setExternal(true, "external_frame");
    EXPECT_EQ(tcp.getExternalFrame(), "external_frame");
    EXPECT_TRUE(tcp.isExternal());
    EXPECT_TRUE(tcp.isTransform());
    EXPECT_TRUE(tcp.getTransform().isApprox(pose, 1e-6));
  }

  {  // TCP as transform
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0, 0, 0.25);

    tesseract_common::ToolCenterPoint tcp(pose);
    EXPECT_FALSE(tcp.empty());
    EXPECT_FALSE(tcp.isString());
    EXPECT_TRUE(tcp.isTransform());
    EXPECT_FALSE(tcp.isExternal());
    EXPECT_TRUE(tcp.getTransform().isApprox(pose, 1e-6));
    EXPECT_ANY_THROW(tcp.getString());
    EXPECT_ANY_THROW(tcp.getExternalFrame());
  }
}

TEST(TesseractCommonUnit, ManipulatorInfo)
{
  // Empty tcp
  tesseract_common::ManipulatorInfo manip_info;
  EXPECT_TRUE(manip_info.empty());
  EXPECT_TRUE(manip_info.tcp.empty());
  EXPECT_TRUE(manip_info.manipulator.empty());
  EXPECT_TRUE(manip_info.manipulator_ik_solver.empty());
  EXPECT_TRUE(manip_info.working_frame.empty());

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0, 0, 0.25);

  tesseract_common::ManipulatorInfo manip_info_override("manipulator");
  manip_info_override.tcp = tesseract_common::ToolCenterPoint(pose);
  manip_info_override.manipulator_ik_solver = "OPWInvKin";
  manip_info_override.working_frame = "tool0";

  manip_info = manip_info.getCombined(manip_info_override);
  EXPECT_FALSE(manip_info.empty());
  EXPECT_TRUE(manip_info.tcp == manip_info_override.tcp);
  EXPECT_EQ(manip_info.manipulator, manip_info_override.manipulator);
  EXPECT_EQ(manip_info.manipulator_ik_solver, manip_info_override.manipulator_ik_solver);
  EXPECT_EQ(manip_info.working_frame, manip_info_override.working_frame);

  // Test empty method
  {
    tesseract_common::ManipulatorInfo manip_info;
    manip_info.manipulator = "manip";
    EXPECT_FALSE(manip_info.empty());
  }

  {
    tesseract_common::ManipulatorInfo manip_info;
    manip_info.manipulator_ik_solver = "manip";
    EXPECT_FALSE(manip_info.empty());
  }

  {
    tesseract_common::ManipulatorInfo manip_info;
    manip_info.working_frame = "manip";
    EXPECT_FALSE(manip_info.empty());
  }

  {
    tesseract_common::ManipulatorInfo manip_info;
    manip_info.tcp = tesseract_common::ToolCenterPoint("manip");
    EXPECT_FALSE(manip_info.empty());
  }
}

TEST(TesseractCommonUnit, serializationToolCenterPoint)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  tesseract_common::ToolCenterPoint tcp(pose);

  {
    std::ofstream os("/tmp/tool_center_point_boost.xml");
    boost::archive::xml_oarchive oa(os);
    oa << BOOST_SERIALIZATION_NVP(tcp);
  }

  tesseract_common::ToolCenterPoint ntcp;
  {
    std::ifstream ifs("/tmp/tool_center_point_boost.xml");
    assert(ifs.good());
    boost::archive::xml_iarchive ia(ifs);

    // restore the schedule from the archive
    ia >> BOOST_SERIALIZATION_NVP(ntcp);
  }

  EXPECT_TRUE(tcp == ntcp);
  EXPECT_FALSE(tcp != ntcp);
}

TEST(TesseractCommonUnit, serializationManipulatorInfo)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  tesseract_common::ManipulatorInfo manip_info("manipulator");
  manip_info.tcp = tesseract_common::ToolCenterPoint(pose);

  {
    std::ofstream os("/tmp/manipulator_info_boost.xml");
    boost::archive::xml_oarchive oa(os);
    oa << BOOST_SERIALIZATION_NVP(manip_info);
  }

  tesseract_common::ManipulatorInfo nmanip_info;
  {
    std::ifstream ifs("/tmp/manipulator_info_boost.xml");
    assert(ifs.good());
    boost::archive::xml_iarchive ia(ifs);

    // restore the schedule from the archive
    ia >> BOOST_SERIALIZATION_NVP(nmanip_info);
  }

  EXPECT_TRUE(manip_info == nmanip_info);
  EXPECT_FALSE(manip_info != nmanip_info);
}

TEST(TesseractCommonUnit, JointStateTest)
{
  std::vector<std::string> joint_names{ "joint_1", "joint_2", "joint_3" };
  Eigen::VectorXd positons = Eigen::VectorXd::Constant(3, 5);
  tesseract_common::JointState joint_state(joint_names, positons);
  EXPECT_TRUE(joint_state.joint_names == joint_names);
  EXPECT_TRUE(joint_state.position.isApprox(positons, 1e-5));
}

TEST(TesseractCommonUnit, serializationJointState)
{
  tesseract_common::JointState joint_state;
  joint_state.joint_names = { "joint_1", "joint_2", "joint_3" };
  joint_state.position = Eigen::VectorXd::Constant(3, 5);
  joint_state.velocity = Eigen::VectorXd::Constant(3, 6);
  joint_state.acceleration = Eigen::VectorXd::Constant(3, 7);
  joint_state.effort = Eigen::VectorXd::Constant(3, 8);
  joint_state.time = 100;

  {
    std::ofstream os("/tmp/joint_state_boost.xml");
    boost::archive::xml_oarchive oa(os);
    oa << BOOST_SERIALIZATION_NVP(joint_state);
  }

  tesseract_common::JointState njoint_state;
  {
    std::ifstream ifs("/tmp/joint_state_boost.xml");
    assert(ifs.good());
    boost::archive::xml_iarchive ia(ifs);

    // restore the schedule from the archive
    ia >> BOOST_SERIALIZATION_NVP(njoint_state);
  }

  EXPECT_TRUE(joint_state == njoint_state);
  EXPECT_FALSE(joint_state != njoint_state);
}

TEST(TesseractCommonUnit, serializationKinematicLimits)
{
  tesseract_common::KinematicLimits limits;
  limits.joint_limits.resize(3, 2);
  limits.joint_limits << -5, 5, -5, 5, -5, 5;
  limits.velocity_limits = Eigen::VectorXd::Constant(3, 6);
  limits.acceleration_limits = Eigen::VectorXd::Constant(3, 7);

  {
    std::ofstream os("/tmp/kinematic_limits_boost.xml");
    boost::archive::xml_oarchive oa(os);
    oa << BOOST_SERIALIZATION_NVP(limits);
  }

  tesseract_common::KinematicLimits nlimits;
  {
    std::ifstream ifs("/tmp/kinematic_limits_boost.xml");
    assert(ifs.good());
    boost::archive::xml_iarchive ia(ifs);

    // restore the schedule from the archive
    ia >> BOOST_SERIALIZATION_NVP(nlimits);
  }

  EXPECT_TRUE(limits == nlimits);
  EXPECT_FALSE(limits != nlimits);
}

TEST(TesseractCommonUnit, serializationVectorXd)
{
  {  // Serialize empty object
    Eigen::VectorXd ev;

    {
      std::ofstream os("/tmp/eigen_vector_xd_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(ev);
    }

    Eigen::VectorXd nev;
    {
      std::ifstream ifs("/tmp/eigen_vector_xd_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nev);
    }
  }

  // Serialize to object which already has data
  for (int i = 0; i < 5; ++i)
  {
    Eigen::VectorXd ev = Eigen::VectorXd::Random(6);

    {
      std::ofstream os("/tmp/eigen_vector_xd_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(ev);
    }

    Eigen::VectorXd nev = Eigen::VectorXd::Random(6);
    {
      std::ifstream ifs("/tmp/eigen_vector_xd_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nev);
    }

    EXPECT_TRUE(ev.isApprox(nev, 1e-5));
  }

  // Serialize to object which already has data and different size
  for (int i = 0; i < 5; ++i)
  {
    Eigen::VectorXd ev = Eigen::VectorXd::Random(6);

    {
      std::ofstream os("/tmp/eigen_vector_xd_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(ev);
    }

    Eigen::VectorXd nev = Eigen::VectorXd::Random(3);
    {
      std::ifstream ifs("/tmp/eigen_vector_xd_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nev);
    }

    EXPECT_TRUE(ev.isApprox(nev, 1e-5));
  }

  // Default use case
  for (int i = 0; i < 5; ++i)
  {
    Eigen::VectorXd ev = Eigen::VectorXd::Random(6);

    {
      std::ofstream os("/tmp/eigen_vector_xd_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(ev);
    }

    Eigen::VectorXd nev;
    {
      std::ifstream ifs("/tmp/eigen_vector_xd_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nev);
    }

    EXPECT_TRUE(ev.isApprox(nev, 1e-5));
  }
}

TEST(TesseractCommonUnit, serializationMatrixX2d)
{
  {  // Serialize empty
    Eigen::MatrixX2d em;

    {
      std::ofstream os("/tmp/eigen_matrix_x2d_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(em);
    }

    Eigen::MatrixX2d nem;
    {
      std::ifstream ifs("/tmp/eigen_matrix_x2d_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nem);
    }

    EXPECT_TRUE(em.isApprox(nem, 1e-5));
  }

  // Serialize to object which already has data
  for (int i = 0; i < 5; ++i)
  {
    Eigen::MatrixX2d em = Eigen::MatrixX2d::Random(4, 2);

    {
      std::ofstream os("/tmp/eigen_matrix_x2d_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(em);
    }

    Eigen::MatrixX2d nem = Eigen::MatrixX2d::Random(4, 2);
    {
      std::ifstream ifs("/tmp/eigen_matrix_x2d_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nem);
    }

    EXPECT_TRUE(em.isApprox(nem, 1e-5));
  }

  // Serialize to object which already has data and different size
  for (int i = 0; i < 5; ++i)
  {
    Eigen::MatrixX2d em = Eigen::MatrixX2d::Random(4, 2);

    {
      std::ofstream os("/tmp/eigen_matrix_x2d_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(em);
    }

    Eigen::MatrixX2d nem = Eigen::MatrixX2d::Random(2, 2);
    {
      std::ifstream ifs("/tmp/eigen_matrix_x2d_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nem);
    }

    EXPECT_TRUE(em.isApprox(nem, 1e-5));
  }

  // Default
  for (int i = 0; i < 5; ++i)
  {
    Eigen::MatrixX2d em = Eigen::MatrixX2d::Random(4, 2);

    {
      std::ofstream os("/tmp/eigen_matrix_x2d_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(em);
    }

    Eigen::MatrixX2d nem;
    {
      std::ifstream ifs("/tmp/eigen_matrix_x2d_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nem);
    }

    EXPECT_TRUE(em.isApprox(nem, 1e-5));
  }
}

TEST(TesseractCommonUnit, serializationIsometry3d)
{
  for (int i = 0; i < 5; ++i)
  {
    Eigen::Isometry3d pose =
        Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::Random().normalized());
    pose.translation() = Eigen::Vector3d::Random();

    {
      std::ofstream os("/tmp/eigen_isometry3d_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(pose);
    }

    Eigen::Isometry3d npose;
    {
      std::ifstream ifs("/tmp/eigen_isometry3d_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(npose);
    }

    EXPECT_TRUE(pose.isApprox(npose, 1e-5));
  }
}

TESSERACT_ANY_EXPORT(tesseract_common::JointState);  // NOLINT

TEST(TesseractCommonUnit, anyUnit)
{
  tesseract_common::Any any_type;
  EXPECT_TRUE(any_type.getType() == std::type_index(typeid(nullptr)));

  tesseract_common::JointState joint_state;
  joint_state.joint_names = { "joint_1", "joint_2", "joint_3" };
  joint_state.position = Eigen::VectorXd::Constant(3, 5);
  joint_state.velocity = Eigen::VectorXd::Constant(3, 6);
  joint_state.acceleration = Eigen::VectorXd::Constant(3, 7);
  joint_state.effort = Eigen::VectorXd::Constant(3, 8);
  joint_state.time = 100;

  any_type = joint_state;
  EXPECT_TRUE(any_type.getType() == std::type_index(typeid(tesseract_common::JointState)));
  EXPECT_TRUE(any_type.as<tesseract_common::JointState>() == joint_state);

  // Check clone
  tesseract_common::Any any_copy = any_type;
  EXPECT_TRUE(any_copy == any_type);

  // Check to make sure it is not making a copy during cast
  auto& any_type_ref1 = any_type.as<tesseract_common::JointState>();
  auto& any_type_ref2 = any_type.as<tesseract_common::JointState>();
  auto& any_copy_ref = any_copy.as<tesseract_common::JointState>();
  EXPECT_TRUE(&any_type_ref1 == &any_type_ref2);
  EXPECT_TRUE(&any_type_ref1 != &any_copy_ref);
  EXPECT_TRUE(&any_type_ref2 != &any_copy_ref);

  const auto& any_type_const_ref1 = any_type.as<tesseract_common::JointState>();
  const auto& any_type_const_ref2 = any_type.as<tesseract_common::JointState>();
  EXPECT_TRUE(&any_type_const_ref1 == &any_type_const_ref2);

  {
    std::ofstream os("/tmp/any_type_boost.xml");
    boost::archive::xml_oarchive oa(os);
    oa << BOOST_SERIALIZATION_NVP(any_type);
  }

  tesseract_common::Any nany_type;
  {
    std::ifstream ifs("/tmp/any_type_boost.xml");
    assert(ifs.good());
    boost::archive::xml_iarchive ia(ifs);

    // restore the schedule from the archive
    ia >> BOOST_SERIALIZATION_NVP(nany_type);
  }

  EXPECT_TRUE(nany_type.getType() == std::type_index(typeid(tesseract_common::JointState)));
  EXPECT_TRUE(nany_type.as<tesseract_common::JointState>() == joint_state);

  // Test bad cast
  EXPECT_ANY_THROW(nany_type.as<tesseract_common::Toolpath>());
}

TEST(TesseractCommonUnit, boundsUnit)
{
  Eigen::VectorXd v = Eigen::VectorXd::Ones(6);
  v = v.array() + std::numeric_limits<float>::epsilon();
  Eigen::MatrixX2d limits(6, 2);
  limits.col(0) = -Eigen::VectorXd::Ones(6);
  limits.col(1) = Eigen::VectorXd::Ones(6);

  EXPECT_TRUE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<float>::epsilon()));
  EXPECT_FALSE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<double>::epsilon()));
  tesseract_common::enforcePositionLimits(v, limits);
  EXPECT_TRUE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<double>::epsilon()));

  v = -Eigen::VectorXd::Ones(6);
  v = v.array() - std::numeric_limits<float>::epsilon();

  EXPECT_TRUE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<float>::epsilon()));
  EXPECT_FALSE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<double>::epsilon()));
  tesseract_common::enforcePositionLimits(v, limits);
  EXPECT_TRUE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<double>::epsilon()));

  // Check that clamp is done correctly on both sides
  v = Eigen::VectorXd::Constant(6, -2);
  EXPECT_FALSE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<double>::epsilon()));
  tesseract_common::enforcePositionLimits(v, limits);
  ASSERT_EQ((v - limits.col(0)).norm(), 0);

  v = Eigen::VectorXd::Constant(6, 2);
  EXPECT_FALSE(tesseract_common::satisfiesPositionLimits(v, limits, std::numeric_limits<double>::epsilon()));
  tesseract_common::enforcePositionLimits(v, limits);
  ASSERT_EQ((v - limits.col(1)).norm(), 0);
}

TEST(TesseractCommonUnit, isIdenticalUnit)
{
  std::vector<std::string> v1{ "a", "b", "c" };
  std::vector<std::string> v2{ "a", "b", "c" };
  EXPECT_TRUE(tesseract_common::isIdentical(v1, v2, false));
  EXPECT_TRUE(tesseract_common::isIdentical(v1, v2, true));

  v2 = { "c", "b", "a" };
  EXPECT_TRUE(tesseract_common::isIdentical(v1, v2, false));
  EXPECT_FALSE(tesseract_common::isIdentical(v1, v2, true));

  v2 = { "a", "b", "d" };
  EXPECT_FALSE(tesseract_common::isIdentical(v1, v2, false));
  EXPECT_FALSE(tesseract_common::isIdentical(v1, v2, true));
}

TEST(TesseractCommonUnit, getTimestampStringUnit)
{
  std::string s1 = tesseract_common::getTimestampString();
  EXPECT_FALSE(s1.empty());
}

TEST(TesseractCommonUnit, reorder)
{
  std::vector<std::vector<Eigen::Index>> checks;
  checks.push_back({ 5, 4, 3, 2, 1, 0 });
  checks.push_back({ 0, 1, 2, 3, 4, 5 });
  checks.push_back({ 3, 2, 4, 1, 5, 0 });
  Eigen::VectorXd v = Eigen::VectorXd::Random(6);

  for (const auto& check : checks)
  {
    Eigen::VectorXd v_copy = v;
    tesseract_common::reorder(v_copy, check);

    for (std::size_t i = 0; i < check.size(); ++i)
    {
      EXPECT_NEAR(v_copy(static_cast<Eigen::Index>(i)), v(check[i]), 1e-8);
    }
  }
}

TEST(TesseractCommonUnit, getTempPathUnit)
{
  std::string s1 = tesseract_common::getTempPath();
  EXPECT_FALSE(s1.empty());
  EXPECT_TRUE(tesseract_common::fs::exists(s1));
}

TEST(TesseractCommonUnit, QueryStringValueUnit)
{
  {
    std::string str = R"(<box>Test</box>)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringValue(element, string_value);
    EXPECT_TRUE(status == tinyxml2::XML_SUCCESS);
    EXPECT_TRUE(string_value == "box");
  }
}

TEST(TesseractCommonUnit, QueryStringTextUnit)
{
  {
    std::string str = R"(<box>Test</box>)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringText(element, string_value);
    EXPECT_TRUE(status == tinyxml2::XML_SUCCESS);
    EXPECT_TRUE(string_value == "Test");
  }

  {
    std::string str = R"(<box></box>)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringText(element, string_value);
    EXPECT_TRUE(status == tinyxml2::XML_NO_ATTRIBUTE);
  }
}

TEST(TesseractCommonUnit, QueryStringAttributeUnit)
{
  {
    std::string str = R"(<box name="test" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(element, "name", string_value);
    EXPECT_TRUE(status == tinyxml2::XML_SUCCESS);
    EXPECT_TRUE(string_value == "test");
  }

  {
    std::string str = R"(<box missing="test" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(element, "name", string_value);
    EXPECT_TRUE(status == tinyxml2::XML_NO_ATTRIBUTE);
  }
}

TEST(TesseractCommonUnit, StringAttributeUnit)
{
  {
    std::string str = R"(<box name="test" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value = tesseract_common::StringAttribute(element, "name", "default");
    EXPECT_TRUE(string_value == "test");
  }

  {
    std::string str = R"(<box name="test" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value = tesseract_common::StringAttribute(element, "missing", "default");
    EXPECT_TRUE(string_value == "default");
  }
}

TEST(TesseractCommonUnit, QueryStringAttributeRequiredUnit)
{
  {
    std::string str = R"(<box name="test" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(element, "name", string_value);
    EXPECT_TRUE(status == tinyxml2::XML_SUCCESS);
    EXPECT_TRUE(string_value == "test");
  }

  {
    std::string str = R"(<box name="test" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    std::string string_value;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttributeRequired(element, "missing", string_value);
    EXPECT_TRUE(status == tinyxml2::XML_NO_ATTRIBUTE);
  }
}

TEST(TesseractCommonUnit, QueryDoubleAttributeRequiredUnit)
{
  {
    std::string str = R"(<box name="1.5" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    double double_value;
    tinyxml2::XMLError status = tesseract_common::QueryDoubleAttributeRequired(element, "name", double_value);
    EXPECT_TRUE(status == tinyxml2::XML_SUCCESS);
    EXPECT_NEAR(double_value, 1.5, 1e-6);
  }

  {
    std::string str = R"(<box name="1.5" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    double double_value;
    tinyxml2::XMLError status = tesseract_common::QueryDoubleAttributeRequired(element, "missing", double_value);
    EXPECT_TRUE(status == tinyxml2::XML_NO_ATTRIBUTE);
  }

  {
    std::string str = R"(<box name="abc" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    double double_value;
    tinyxml2::XMLError status = tesseract_common::QueryDoubleAttributeRequired(element, "name", double_value);
    EXPECT_TRUE(status == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE);
  }
}

TEST(TesseractCommonUnit, QueryIntAttributeRequiredUnit)
{
  {
    std::string str = R"(<box name="1" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    int int_value;
    tinyxml2::XMLError status = tesseract_common::QueryIntAttributeRequired(element, "name", int_value);
    EXPECT_TRUE(status == tinyxml2::XML_SUCCESS);
    EXPECT_NEAR(int_value, 1, 1e-6);
  }

  {
    std::string str = R"(<box name="1.5" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    int int_value;
    tinyxml2::XMLError status = tesseract_common::QueryIntAttributeRequired(element, "missing", int_value);
    EXPECT_TRUE(status == tinyxml2::XML_NO_ATTRIBUTE);
  }

  {
    std::string str = R"(<box name="abc" />)";
    tinyxml2::XMLDocument xml_doc;
    EXPECT_TRUE(xml_doc.Parse(str.c_str()) == tinyxml2::XML_SUCCESS);

    tinyxml2::XMLElement* element = xml_doc.FirstChildElement("box");
    EXPECT_TRUE(element != nullptr);

    int int_value;
    tinyxml2::XMLError status = tesseract_common::QueryIntAttributeRequired(element, "name", int_value);
    EXPECT_TRUE(status == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE);
  }
}

// sample function that catches an exception and wraps it in a nested exception
void runThrowNestedException()
{
  try
  {
    throw std::runtime_error("failed");
  }
  catch (...)
  {
    std::throw_with_nested(std::runtime_error("runThrowNestedException() failed"));
  }
}

TEST(TesseractCommonUnit, printNestedExceptionUnit)
{
  try
  {
    runThrowNestedException();
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
  }
}

TEST(TesseractCommonUnit, almostEqualRelativeAndAbsUnit)
{
  double a = 1e-5;
  double b = 0;
  EXPECT_FALSE(tesseract_common::almostEqualRelativeAndAbs(a, b));

  a = 1e-7;
  EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(a, b));

  a = 100000000000000.01;
  b = 100000000000000;
  EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(a, b));

  a = 100000000000000.1;
  b = 100000000000000;
  EXPECT_FALSE(tesseract_common::almostEqualRelativeAndAbs(a, b));

  Eigen::VectorXd v1 = Eigen::VectorXd::Constant(3, 1e-5);
  Eigen::VectorXd v2 = Eigen::VectorXd::Constant(3, 0);
  EXPECT_FALSE(tesseract_common::almostEqualRelativeAndAbs(v1, v2));

  v1 = Eigen::VectorXd::Constant(3, 1e-7);
  EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(v1, v2));

  v1 = Eigen::VectorXd::Constant(3, 100000000000000.01);
  v2 = Eigen::VectorXd::Constant(3, 100000000000000);
  EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(v1, v2));

  v1 = Eigen::VectorXd::Constant(3, 100000000000000.1);
  v2 = Eigen::VectorXd::Constant(3, 100000000000000);
  EXPECT_FALSE(tesseract_common::almostEqualRelativeAndAbs(v1, v2));

  v2 = Eigen::VectorXd::Constant(1, 100000000000000);
  EXPECT_FALSE(tesseract_common::almostEqualRelativeAndAbs(v1, v2));

  EXPECT_TRUE(tesseract_common::almostEqualRelativeAndAbs(Eigen::VectorXd(), Eigen::VectorXd()));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
