#ifndef TESSERACT_COLLISION_COLLISION_BOX_BOX_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_BOX_BOX_UNIT_HPP

#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>

namespace tesseract_collision
{
namespace test_suite
{
namespace detail
{
inline void addCollisionObjects(DiscreteContactManager& checker, bool use_convex_mesh = false)
{
  //////////////////////
  // Add box to checker
  //////////////////////
  CollisionShapePtr box = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);

  checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses);

  /////////////////////////////////////////////
  // Add thin box to checker which is disabled
  /////////////////////////////////////////////
  CollisionShapePtr thin_box = std::make_shared<tesseract_geometry::Box>(0.1, 1, 1);
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, false);

  /////////////////////////////////////////////////////////////////
  // Add second box to checker. If use_convex_mesh = true then this
  // box will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr second_box;

  if (use_convex_mesh)
  {
    tesseract_common::VectorVector3d mesh_vertices;
    Eigen::VectorXi mesh_faces;
    EXPECT_GT(loadSimplePlyFile(std::string(TEST_SUITE_DATA_DIR) + "/box_2m.ply", mesh_vertices, mesh_faces), 0);

    // This is required because convex hull cannot have multiple faces on the same plane.
    auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
    auto ch_faces = std::make_shared<Eigen::VectorXi>();
    int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
    second_box = std::make_shared<tesseract_geometry::ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
  }
  else
  {
    second_box = std::make_shared<tesseract_geometry::Box>(2, 2, 2);
  }

  Eigen::Isometry3d second_box_pose;
  second_box_pose.setIdentity();

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(second_box);
  obj3_poses.push_back(second_box_pose);

  checker.addCollisionObject("second_box_link", 0, obj3_shapes, obj3_poses);
}
}  // namespace detail

inline void runTest(DiscreteContactManager& checker, bool use_convex_mesh = false)
{
  // Add collision objects
  detail::addCollisionObjects(checker, use_convex_mesh);

  //////////////////////////////////////
  // Test when object is inside another
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "box_link", "second_box_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["box_link"].translation()(0) = 0.2;
  location["box_link"].translation()(1) = 0.1;
  location["second_box_link"] = Eigen::Isometry3d::Identity();

  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactTestType::CLOSEST);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -1.30, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], -0.3, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * -1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["box_link"].translation() = Eigen::Vector3d(1.60, 0, 0);
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  checker.setCollisionObjectsTransform(location);
  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  checker.setContactDistanceThreshold(0.25);
  checker.contactTest(result, ContactTestType::CLOSEST);
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.1, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 1.1, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * -1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}
}  // namespace test_suite
}  // namespace tesseract_collision
#endif  // COLLISION_BOX_BOX_UNIT_HPP
