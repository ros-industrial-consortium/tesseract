/**
 * @file ompl_planner_tests.cpp
 * @brief This contains unit test for the tesseract descartes planner
 *
 * @author Levi Armstrong, Jonathan Meyer
 * @date September 16, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/filesystem/path.hpp>

#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>

#include <functional>
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/conversions.h>
#include <tesseract/tesseract.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/ompl/ompl_freespace_planner.h>

using namespace tesseract;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;
using namespace tesseract_geometry;
using namespace tesseract_kinematics;
using namespace tesseract_motion_planners;

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

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

static void addBox(tesseract_environment::Environment& env)
{
  Link link_1("box_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.4, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Box>(0.5, 0.001, 0.5);
  link_1.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_1.collision.push_back(collision);

  Joint joint_1("joint_n1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = link_1.getName();
  joint_1.type = JointType::FIXED;

  env.addLink(link_1, joint_1);
}

template <typename PlannerType>
class OMPLTestFixture : public ::testing::Test
{
public:
  using ::testing::Test::Test;
  tesseract_motion_planners::OMPLFreespacePlanner<PlannerType> ompl_planner;
};

typedef ::testing::Types<ompl::geometric::SBL,
                         ompl::geometric::PRM,
                         ompl::geometric::PRMstar,
                         ompl::geometric::LazyPRMstar,
                         ompl::geometric::EST,
                         ompl::geometric::LBKPIECE1,
                         ompl::geometric::BKPIECE1,
                         ompl::geometric::KPIECE1,
                         ompl::geometric::RRT,
                         ompl::geometric::RRTConnect,
//                         ompl::geometric::RRTstar,
                         ompl::geometric::TRRT,
                         ompl::geometric::SPARS>
    Implementations;

TYPED_TEST_CASE(OMPLTestFixture, Implementations);

TYPED_TEST(OMPLTestFixture, OMPLFreespacePlannerUnit)
{
  // Step 1: Load scene and srdf
  ResourceLocatorFn locator = locateResource;
  Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));

  // Step 2: Add box to environment
  addBox(*(tesseract->getEnvironment()));

  // Step 3: Create ompl planner config and populate it
  auto kin = tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
  std::vector<double> swp = { -1.2, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  std::vector<double> ewp = { 1.2, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0 };

  tesseract_motion_planners::OMPLFreespacePlannerConfig<TypeParam> ompl_config;

  ompl_config.start_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
  ompl_config.end_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());
  ompl_config.tesseract = tesseract;
  ompl_config.manipulator = "manipulator";
  ompl_config.collision_safety_margin = 0.01;
  ompl_config.planning_time = 5.0;
  ompl_config.num_threads = 4;
  ompl_config.max_solutions = 4;

  // Set the planner configuration
  this->ompl_planner.setConfiguration(ompl_config);

  tesseract_motion_planners::PlannerResponse ompl_planning_response;
  tesseract_common::StatusCode status = this->ompl_planner.solve(ompl_planning_response);

  EXPECT_TRUE(status);

  // Check for start state in collision error
  swp = { 0, 0.7, 0.0, 0, 0.0, 0, 0.0 };
  ompl_config.start_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());

  this->ompl_planner.setConfiguration(ompl_config);
  status = this->ompl_planner.solve(ompl_planning_response);

  EXPECT_FALSE(status);

  // Check for start state in collision error
  swp = { -1.2, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  ewp = { 0, 0.7, 0.0, 0, 0.0, 0, 0.0 };
  ompl_config.start_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
  ompl_config.end_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());

  this->ompl_planner.setConfiguration(ompl_config);
  status = this->ompl_planner.solve(ompl_planning_response);

  EXPECT_FALSE(status);

  // Reset start and end waypoints
  swp = { -1.2, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  ewp = { 1.2, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0 };
  ompl_config.start_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(swp, kin->getJointNames());
  ompl_config.end_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(ewp, kin->getJointNames());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
