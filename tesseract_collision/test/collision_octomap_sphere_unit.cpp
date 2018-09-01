#include "tesseract_collision/bullet/bullet_discrete_managers.h"
#include <octomap/octomap.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

void addCollisionObjects(tesseract::DiscreteContactManagerBase& checker, bool use_convex_mesh = false)
{
  /////////////////////////////////////////////////////////////////
  // Add Octomap
  /////////////////////////////////////////////////////////////////
  std::string path = ros::package::getPath("tesseract_collision") + "/test/blender_monkey.bt";
  std::shared_ptr<octomap::OcTree> ot(new octomap::OcTree(path));
  shapes::ShapePtr dense_octomap(new shapes::OcTree(ot));
  Eigen::Isometry3d octomap_pose;
  octomap_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  tesseract::VectorIsometry3d obj1_poses;
  tesseract::CollisionObjectTypeVector obj1_types;
  obj1_shapes.push_back(dense_octomap);
  obj1_poses.push_back(octomap_pose);
  obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("octomap_link", 0, obj1_shapes, obj1_poses, obj1_types);

  /////////////////////////////////////////////////////////////////
  // Add sphere to checker. If use_convex_mesh = true then this
  // sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  shapes::ShapePtr sphere;
  if (use_convex_mesh)
    sphere.reset(shapes::createMeshFromResource("package://tesseract_collision/test/sphere_p25m.stl"));
  else
    sphere.reset(new shapes::Sphere(0.25));

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  tesseract::VectorIsometry3d obj2_poses;
  tesseract::CollisionObjectTypeVector obj2_types;
  obj2_shapes.push_back(sphere);
  obj2_poses.push_back(sphere_pose);

  if (use_convex_mesh)
    obj2_types.push_back(tesseract::CollisionObjectType::ConvexHull);
  else
    obj2_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("sphere_link", 0, obj2_shapes, obj2_poses, obj2_types);
}

void runTest(tesseract::DiscreteContactManagerBase& checker)
{
  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  tesseract::ContactRequest req;
  req.link_names.push_back("octomap_link");
  req.link_names.push_back("sphere_link");
  req.contact_distance = 0.1;
  req.type = tesseract::ContactRequestType::CLOSEST;
  checker.setContactRequest(req);

  // Set the collision object transforms
  tesseract::TransformMap location;
  location["octomap_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere_link"].translation() = Eigen::Vector3d(0, 0, 1);
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ros::WallTime start_time = ros::WallTime::now();
  tesseract::ContactResultMap result;
  for (auto i = 0; i < 10; ++i)
  {
    result.clear();
    checker.contactTest(result);
  }
  ros::WallTime end_time = ros::WallTime::now();
  ROS_INFO_STREAM("DT: " << (end_time - start_time).toSec());

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -0.25, 0.001);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionOctomapSphereUnit)
{
  tesseract::BulletDiscreteSimpleManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

//TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionBoxSphereConvexHullUnit)
//{
//  tesseract::BulletDiscreteSimpleManager checker;
//  addCollisionObjects(checker, true);
//  runTest(checker);
//}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionOctomapSphereUnit)
{
  tesseract::BulletDiscreteBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker);
}

//TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionBoxSphereConvexHullUnit)
//{
//  tesseract::BulletDiscreteBVHManager checker;
//  addCollisionObjects(checker, true);
//  runTest(checker);
//}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
