#include <console_bridge/console.h>
#include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"

using namespace tesseract_collision;
using namespace tesseract_geometry;

std::string toString(const Eigen::MatrixXd& a)
{
  std::stringstream ss;
  ss << a;
  return ss.str();
}

std::string toString(bool b) { return b ? "true" : "false"; }

int main(int /*argc*/, char** /*argv*/)
{
  // documentation:start:1: Create Collision Manager
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  // documentation:end:1: Create Collision Manager

  // documentation:start:2: Add box to checker
  // Create a box
  CollisionShapePtr box = std::make_shared<Box>(1, 1, 1);
  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);

  // Add box to checker in enabled state
  checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses);
  // documentation:end:2: Add box to checker

  // documentation:start:3: Add thin box
  // Create a thin box
  CollisionShapePtr thin_box = std::make_shared<Box>(0.1, 1, 1);
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);

  // Add thin box to checker in disabled state
  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, false);
  // documentation:end:3: Add thin box

  // documentation:start:4: Add convex hull
  // Add second box to checker, but convert to convex hull mesh
  CollisionShapePtr second_box;

  tesseract_common::VectorVector3d mesh_vertices;
  Eigen::VectorXi mesh_faces;
  loadSimplePlyFile(std::string(TESSERACT_SUPPORT_DIR) + "/meshes/box_2m.ply", mesh_vertices, mesh_faces);

  // This is required because convex hull cannot have multiple faces on the same plane.
  auto ch_verticies = std::make_shared<tesseract_common::VectorVector3d>();
  auto ch_faces = std::make_shared<Eigen::VectorXi>();
  int ch_num_faces = createConvexHull(*ch_verticies, *ch_faces, mesh_vertices);
  second_box = std::make_shared<ConvexMesh>(ch_verticies, ch_faces, ch_num_faces);
  // documentation:end:4: Add convex hull

  // documentation:start:5: Add convex hull collision
  Eigen::Isometry3d second_box_pose;
  second_box_pose.setIdentity();

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(second_box);
  obj3_poses.push_back(second_box_pose);

  checker.addCollisionObject("second_box_link", 0, obj3_shapes, obj3_poses);
  // documentation:end:5: Add convex hull collision

  CONSOLE_BRIDGE_logInform("Test when object is inside another");
  // documentation:start:6: Set active collision object
  checker.setActiveCollisionObjects({ "box_link", "second_box_link" });
  // documentation:end:6: Set active collision object

  // documentation:start:7: Set contact distance threshold
  checker.setCollisionMarginData(CollisionMarginData(0.1));
  // documentation:end:7: Set contact distance threshold

  // documentation:start:8: Set collision object transform
  // Set the collision object transforms
  tesseract_common::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["box_link"].translation()(0) = 0.2;
  location["box_link"].translation()(1) = 0.1;
  location["second_box_link"] = Eigen::Isometry3d::Identity();

  checker.setCollisionObjectsTransform(location);
  // documentation:end:8: Set collision object transform

  // documentation:start:9: Perform collision check
  ContactResultMap result;
  ContactRequest request(ContactTestType::CLOSEST);
  checker.contactTest(result, request);

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  CONSOLE_BRIDGE_logInform("Has collision: %s", toString(result_vector.empty()).c_str());
  CONSOLE_BRIDGE_logInform("Distance: %f", result_vector[0].distance);
  CONSOLE_BRIDGE_logInform("Link %s nearest point: %s",
                           result_vector[0].link_names[0].c_str(),
                           toString(result_vector[0].nearest_points[0]).c_str());
  CONSOLE_BRIDGE_logInform("Link %s nearest point: %s",
                           result_vector[0].link_names[1].c_str(),
                           toString(result_vector[0].nearest_points[1]).c_str());
  CONSOLE_BRIDGE_logInform("Direction to move Link %s out of collision with Link %s: %s",
                           result_vector[0].link_names[0].c_str(),
                           result_vector[0].link_names[1].c_str(),
                           toString(result_vector[0].normal).c_str());
  // documentation:end:9: Perform collision check

  // documentation:start:10: Set collision object transform
  CONSOLE_BRIDGE_logInform("Test object is out side the contact distance");
  location["box_link"].translation() = Eigen::Vector3d(1.60, 0, 0);
  checker.setCollisionObjectsTransform(location);
  // documentation:end:10: Set collision object transform

  // documentation:start:11: Perform collision check
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  // Check for collision after moving object
  checker.contactTest(result, request);
  flattenResults(std::move(result), result_vector);
  CONSOLE_BRIDGE_logInform("Has collision: %s", toString(result_vector.empty()).c_str());
  // documentation:end:11: Perform collision check

  // documentation:start:12: Change contact distance threshold
  // Set higher contact distance threshold
  checker.setDefaultCollisionMarginData(0.25);
  // documentation:end:12: Change contact distance threshold

  // documentation:start:13: Perform collision check
  CONSOLE_BRIDGE_logInform("Test object inside the contact distance");
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  // Check for contact with new threshold
  checker.contactTest(result, request);
  flattenResults(std::move(result), result_vector);

  CONSOLE_BRIDGE_logInform("Has collision: %s", toString(result_vector.empty()).c_str());
  CONSOLE_BRIDGE_logInform("Distance: %f", result_vector[0].distance);
  CONSOLE_BRIDGE_logInform("Link %s nearest point: %s",
                           result_vector[0].link_names[0].c_str(),
                           toString(result_vector[0].nearest_points[0]).c_str());
  CONSOLE_BRIDGE_logInform("Link %s nearest point: %s",
                           result_vector[0].link_names[1].c_str(),
                           toString(result_vector[0].nearest_points[1]).c_str());
  CONSOLE_BRIDGE_logInform("Direction to move Link %s further from Link %s: %s",
                           result_vector[0].link_names[0].c_str(),
                           result_vector[0].link_names[1].c_str(),
                           toString(result_vector[0].normal).c_str());
  // documentation:end:13: Perform collision check
}
