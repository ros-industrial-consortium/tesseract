#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <numeric>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/types.h>

using namespace tesseract_collision;

TEST(TesseractCollisionUnit, CollisionMarginDataUnit)  // NOLINT
{
  double tol = std::numeric_limits<double>::epsilon();

  {  // Test Default Constructor
    CollisionMarginData data;
    EXPECT_NEAR(data.getDefaultCollisionMarginData(), 0, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), 0, tol);
    EXPECT_NEAR(data.getPairCollisionMarginData("link_1", "link_2"), 0, tol);
  }

  {  // Test construction with non zero default distance
    double default_margin = 0.0254;
    CollisionMarginData data(default_margin);
    EXPECT_NEAR(data.getDefaultCollisionMarginData(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getPairCollisionMarginData("link_1", "link_2"), default_margin, tol);
  }

  {  // Test changing default margin
    double default_margin = 0.0254;
    CollisionMarginData data;
    data.setDefaultCollisionMarginData(default_margin);
    EXPECT_NEAR(data.getDefaultCollisionMarginData(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getPairCollisionMarginData("link_1", "link_2"), default_margin, tol);
  }

  {  // Test adding pair margin larger than default
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setPairCollisionMarginData("link_1", "link_2", pair_margin);
    EXPECT_NEAR(data.getDefaultCollisionMarginData(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), pair_margin, tol);
    EXPECT_NEAR(data.getPairCollisionMarginData("link_1", "link_2"), pair_margin, tol);
  }

  {  // Test adding pair margin less than default
    double default_margin = 0.0254;
    double pair_margin = 0.01;
    CollisionMarginData data(default_margin);
    data.setPairCollisionMarginData("link_1", "link_2", pair_margin);
    EXPECT_NEAR(data.getDefaultCollisionMarginData(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getPairCollisionMarginData("link_1", "link_2"), pair_margin, tol);
  }

  {  // Test setting default larger than the current max margin
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setPairCollisionMarginData("link_1", "link_2", pair_margin);

    default_margin = 2 * pair_margin;
    data.setDefaultCollisionMarginData(default_margin);
    EXPECT_NEAR(data.getDefaultCollisionMarginData(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getPairCollisionMarginData("link_1", "link_2"), pair_margin, tol);
  }

  {  // Test setting pair_margin larger than default and then set it lower so the max should be the default
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    CollisionMarginData data(default_margin);
    data.setPairCollisionMarginData("link_1", "link_2", pair_margin);
    data.setPairCollisionMarginData("link_1", "link_2", default_margin);
    EXPECT_NEAR(data.getDefaultCollisionMarginData(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), default_margin, tol);
    EXPECT_NEAR(data.getPairCollisionMarginData("link_1", "link_2"), default_margin, tol);
  }

  {  // Test setting default larger than pair the change to lower than pair and the max should be the pair
    double default_margin = 0.05;
    double pair_margin = 0.0254;
    CollisionMarginData data(default_margin);
    data.setPairCollisionMarginData("link_1", "link_2", pair_margin);

    default_margin = 0.0;
    data.setDefaultCollisionMarginData(default_margin);
    EXPECT_NEAR(data.getDefaultCollisionMarginData(), default_margin, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), pair_margin, tol);
    EXPECT_NEAR(data.getPairCollisionMarginData("link_1", "link_2"), pair_margin, tol);
  }

  {  // Test increment positive
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    double increment = 0.01;
    CollisionMarginData data(default_margin);
    data.setPairCollisionMarginData("link_1", "link_2", pair_margin);
    data.incrementMargins(increment);
    EXPECT_NEAR(data.getDefaultCollisionMarginData(), default_margin + increment, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), pair_margin + increment, tol);
    EXPECT_NEAR(data.getPairCollisionMarginData("link_1", "link_2"), pair_margin + increment, tol);
  }

  {  // Test increment negative
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    double increment = -0.01;
    CollisionMarginData data(default_margin);
    data.setPairCollisionMarginData("link_1", "link_2", pair_margin);
    data.incrementMargins(increment);
    EXPECT_NEAR(data.getDefaultCollisionMarginData(), default_margin + increment, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), pair_margin + increment, tol);
    EXPECT_NEAR(data.getPairCollisionMarginData("link_1", "link_2"), pair_margin + increment, tol);
  }

  {  // Test scale > 1
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    double scale = 1.5;
    CollisionMarginData data(default_margin);
    data.setPairCollisionMarginData("link_1", "link_2", pair_margin);
    data.scaleMargins(scale);
    EXPECT_NEAR(data.getDefaultCollisionMarginData(), default_margin * scale, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), pair_margin * scale, tol);
    EXPECT_NEAR(data.getPairCollisionMarginData("link_1", "link_2"), pair_margin * scale, tol);
  }

  {  // Test scale < 1
    double default_margin = 0.0254;
    double pair_margin = 0.5;
    double scale = 0.5;
    CollisionMarginData data(default_margin);
    data.setPairCollisionMarginData("link_1", "link_2", pair_margin);
    data.scaleMargins(scale);
    EXPECT_NEAR(data.getDefaultCollisionMarginData(), default_margin * scale, tol);
    EXPECT_NEAR(data.getMaxCollisionMargin(), pair_margin * scale, tol);
    EXPECT_NEAR(data.getPairCollisionMarginData("link_1", "link_2"), pair_margin * scale, tol);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
