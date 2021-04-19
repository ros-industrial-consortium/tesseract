#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/collision.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_collision)  // NOLINT
{
  std::shared_ptr<tesseract_scene_graph::SimpleResourceLocator> resource_locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);

  {
    std::string str = R"(<collision extra="0 0 0">
                           <origin xyz="1 2 3" rpy="0 0 0" />
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>
                         </collision>)";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(
        elem, &tesseract_urdf::parseCollision, str, "collision", resource_locator, 2));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_FALSE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<collision>
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>"
                         </collision>)";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(
        elem, &tesseract_urdf::parseCollision, str, "collision", resource_locator, 2));
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_TRUE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<collision>
                           <geometry>
                             <mesh filename="package://tesseract_support/meshes/box_box.dae"/>
                           </geometry>"
                         </collision>)";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    EXPECT_TRUE(runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(
        elem, &tesseract_urdf::parseCollision, str, "collision", resource_locator, 2));
    EXPECT_TRUE(elem.size() == 2);
    EXPECT_TRUE(elem[0]->geometry != nullptr);
    EXPECT_TRUE(elem[0]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem[1]->geometry != nullptr);
    EXPECT_TRUE(elem[1]->origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<collision extra="0 0 0">
                           <origin xyz="1 2 3 5" rpy="0 0 0" />
                           <geometry>
                             <box size="1 2 3" />
                           </geometry>
                         </collision>)";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(
        elem, &tesseract_urdf::parseCollision, str, "collision", resource_locator, 2));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<collision>
                           <geometry>
                             <box size="1 2 3 4" />
                           </geometry>"
                         </collision>)";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(
        elem, &tesseract_urdf::parseCollision, str, "collision", resource_locator, 2));
    EXPECT_TRUE(elem.empty());
  }

  {
    std::string str = R"(<collision>
                         </collision>)";
    std::vector<tesseract_scene_graph::Collision::Ptr> elem;
    EXPECT_FALSE(runTest<std::vector<tesseract_scene_graph::Collision::Ptr>>(
        elem, &tesseract_urdf::parseCollision, str, "collision", resource_locator, 2));
    EXPECT_TRUE(elem.empty());
  }
}
