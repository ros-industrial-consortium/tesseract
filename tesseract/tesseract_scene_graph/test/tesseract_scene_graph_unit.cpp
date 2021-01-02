#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
//#include <boost/graph/filtered_graph.hpp>
#include <iostream>
#include <fstream>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_scene_graph/srdf_model.h>
#include <tesseract_scene_graph/parser/kdl_parser.h>
#include <tesseract_scene_graph/srdf/group_opw_kinematics.h>

// getLinks and getJoint use an internal map so need to check against graph
void checkSceneGraph(tesseract_scene_graph::SceneGraph& scene_graph)
{
  using namespace tesseract_scene_graph;

  std::vector<Link::ConstPtr> links = scene_graph.getLinks();
  std::vector<Link::ConstPtr> check_links;
  SceneGraph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(scene_graph); i != iend; ++i)
    check_links.push_back(boost::get(boost::vertex_link, scene_graph)[*i]);

  EXPECT_TRUE(links.size() == check_links.size());

  for (const auto& l : links)
  {
    auto it = std::find_if(
        check_links.begin(), check_links.end(), [&](const Link::ConstPtr& p) { return p.get() == l.get(); });
    EXPECT_TRUE(it != check_links.end());
  }
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g;

  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");

  g.addLink(std::move(link_1));
  g.addLink(std::move(link_2));
  g.addLink(std::move(link_3));
  g.addLink(std::move(link_4));
  g.addLink(std::move(link_5));

  Joint joint_1("joint_1");
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::FIXED;
  g.addJoint(std::move(joint_1));

  Joint joint_2("joint_2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_2";
  joint_2.child_link_name = "link_3";
  joint_2.type = JointType::PLANAR;
  g.addJoint(std::move(joint_2));

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_3";
  joint_3.child_link_name = "link_4";
  joint_3.type = JointType::FLOATING;
  g.addJoint(std::move(joint_3));

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4.parent_link_name = "link_2";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_4));

  // Check getAdjacentLinkNames Method
  std::vector<std::string> adjacent_links = g.getAdjacentLinkNames("link_3");
  EXPECT_TRUE(adjacent_links.size() == 1);
  EXPECT_TRUE(adjacent_links[0] == "link_4");

  // Check getInvAdjacentLinkNames Method
  std::vector<std::string> inv_adjacent_links = g.getInvAdjacentLinkNames("link_3");
  EXPECT_TRUE(inv_adjacent_links.size() == 1);
  EXPECT_TRUE(inv_adjacent_links[0] == "link_2");

  // Check getLinkChildrenNames
  std::vector<std::string> child_link_names = g.getLinkChildrenNames("link_5");
  EXPECT_TRUE(child_link_names.empty());

  child_link_names = g.getLinkChildrenNames("link_3");
  EXPECT_TRUE(child_link_names.size() == 1);
  EXPECT_TRUE(child_link_names[0] == "link_4");

  child_link_names = g.getLinkChildrenNames("link_2");
  EXPECT_TRUE(child_link_names.size() == 3);
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_3") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_4") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_5") != child_link_names.end());

  // Check getJointChildrenNames
  child_link_names = g.getJointChildrenNames("joint_4");
  EXPECT_TRUE(child_link_names.size() == 1);
  EXPECT_TRUE(child_link_names[0] == "link_5");

  child_link_names = g.getJointChildrenNames("joint_3");
  EXPECT_TRUE(child_link_names.size() == 1);
  EXPECT_TRUE(child_link_names[0] == "link_4");

  child_link_names = g.getJointChildrenNames("joint_1");
  EXPECT_TRUE(child_link_names.size() == 4);
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_2") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_3") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_4") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_5") != child_link_names.end());

  // Check Graph
  checkSceneGraph(g);

  // Save Graph
  g.saveDOT(tesseract_common::getTempPath() + "graph_acyclic_tree_example.dot");

  // Test if the graph is Acyclic
  std::cout << "Is Acyclic: " << g.isAcyclic() << std::endl;
  EXPECT_TRUE(g.isAcyclic());

  // Test if the graph is Tree
  std::cout << "Is Tree: " << g.isTree() << std::endl;
  EXPECT_TRUE(g.isTree());

  // Test for unused links
  Link link_6("link_6");
  g.addLink(std::move(link_6));
  std::cout << "Free Link, Is Tree: " << g.isTree() << std::endl;
  EXPECT_FALSE(g.isTree());

  // Check Graph
  checkSceneGraph(g);

  g.removeLink("link_6");
  std::cout << "Free Link Removed, Is Tree: " << g.isTree() << std::endl;
  EXPECT_TRUE(g.isTree());

  // Check Graph
  checkSceneGraph(g);

  Joint joint_5("joint_5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.5;
  joint_5.parent_link_name = "link_5";
  joint_5.child_link_name = "link_4";
  joint_5.type = JointType::CONTINUOUS;
  g.addJoint(std::move(joint_5));

  // Check Graph
  checkSceneGraph(g);

  // Save Graph
  g.saveDOT(tesseract_common::getTempPath() + "graph_acyclic_not_tree_example.dot");

  // Test if the graph is Acyclic
  std::cout << "Is Acyclic: " << g.isAcyclic() << std::endl;
  EXPECT_TRUE(g.isAcyclic());

  // Test if the graph is Tree
  std::cout << "Is Tree: " << g.isTree() << std::endl;
  EXPECT_FALSE(g.isTree());

  Joint joint_6("joint_6");
  joint_6.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6.parent_link_name = "link_5";
  joint_6.child_link_name = "link_1";
  joint_6.type = JointType::CONTINUOUS;
  g.addJoint(std::move(joint_6));

  // Check Graph
  checkSceneGraph(g);

  // Save Graph
  g.saveDOT(tesseract_common::getTempPath() + "graph_cyclic_not_tree_example.dot");

  // Test if the graph is Acyclic
  std::cout << "Is Acyclic: " << g.isAcyclic() << std::endl;
  EXPECT_FALSE(g.isAcyclic());

  // Test if the graph is Tree
  std::cout << "Is Tree: " << g.isTree() << std::endl;
  EXPECT_FALSE(g.isTree());

  // Get Shortest Path
  SceneGraph::Path path = g.getShortestPath("link_1", "link_4");

  // Todo:: Look at using filtered graph for chains and subgraphs
  // boost::filtered_graph<

  std::cout << path << std::endl;
  EXPECT_TRUE(path.first.size() == 4);
  EXPECT_TRUE(std::find(path.first.begin(), path.first.end(), "link_1") != path.first.end());
  EXPECT_TRUE(std::find(path.first.begin(), path.first.end(), "link_2") != path.first.end());
  EXPECT_TRUE(std::find(path.first.begin(), path.first.end(), "link_3") != path.first.end());
  EXPECT_TRUE(std::find(path.first.begin(), path.first.end(), "link_4") != path.first.end());
  EXPECT_TRUE(path.second.size() == 3);
  EXPECT_TRUE(std::find(path.second.begin(), path.second.end(), "joint_1") != path.second.end());
  EXPECT_TRUE(std::find(path.second.begin(), path.second.end(), "joint_2") != path.second.end());
  EXPECT_TRUE(std::find(path.second.begin(), path.second.end(), "joint_3") != path.second.end());

  std::cout << (g.getName().c_str()) << std::endl;

  // Should throw since this is a directory and not a file
  EXPECT_ANY_THROW(g.saveDOT(tesseract_common::getTempPath()));
}

std::string locateResource(const std::string& url)
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

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

TEST(TesseractSceneGraphUnit, LoadSRDFUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;

  std::string srdf_file = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf";

  ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
  SceneGraph g;

  g.setName("kuka_lbr_iiwa_14_r820");

  Link base_link("base_link");
  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");
  Link link_6("link_6");
  Link link_7("link_7");
  Link tool0("tool0");

  g.addLink(std::move(base_link));
  g.addLink(std::move(link_1));
  g.addLink(std::move(link_2));
  g.addLink(std::move(link_3));
  g.addLink(std::move(link_4));
  g.addLink(std::move(link_5));
  g.addLink(std::move(link_6));
  g.addLink(std::move(link_7));
  g.addLink(std::move(tool0));

  Joint joint_1("joint_a1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = "link_1";
  joint_1.type = JointType::FIXED;
  g.addJoint(std::move(joint_1));

  Joint joint_2("joint_a2");
  joint_2.parent_link_name = "link_1";
  joint_2.child_link_name = "link_2";
  joint_2.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_2));

  Joint joint_3("joint_a3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_2";
  joint_3.child_link_name = "link_3";
  joint_3.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_3));

  Joint joint_4("joint_a4");
  joint_4.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_4.parent_link_name = "link_3";
  joint_4.child_link_name = "link_4";
  joint_4.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_4));

  Joint joint_5("joint_a5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_4";
  joint_5.child_link_name = "link_5";
  joint_5.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_5));

  Joint joint_6("joint_a6");
  joint_6.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6.parent_link_name = "link_5";
  joint_6.child_link_name = "link_6";
  joint_6.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_6));

  Joint joint_7("joint_a7");
  joint_7.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_7.parent_link_name = "link_6";
  joint_7.child_link_name = "link_7";
  joint_7.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_7));

  Joint joint_tool0("joint_tool0");
  joint_tool0.parent_link_name = "link_7";
  joint_tool0.child_link_name = "tool0";
  joint_tool0.type = JointType::FIXED;
  g.addJoint(std::move(joint_tool0));

  SRDFModel srdf;
  EXPECT_TRUE(srdf.initFile(g, srdf_file));

  processSRDFAllowedCollisions(g, srdf);

  AllowedCollisionMatrix::ConstPtr acm = g.getAllowedCollisionMatrix();

  // collision between link1 and link2 should be allowed
  EXPECT_TRUE(acm->isCollisionAllowed("link_1", "link_2"));

  // collision between link1 and link2 should be allowed
  EXPECT_FALSE(acm->isCollisionAllowed("base_link", "link_5"));

  g.removeAllowedCollision("link_1", "link_2");
  // now collision link1 and link2 is not allowed anymore
  EXPECT_FALSE(acm->isCollisionAllowed("link_1", "link_2"));

  g.clearAllowedCollisions();
  EXPECT_EQ(acm->getAllAllowedCollisions().size(), 0);
}

TEST(TesseractSceneGraphUnit, LoadSRDFOPWKinematicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g;

  std::string xml_string =
      R"(<robot name="abb_irb2400">
           <group_opw group="manipulator" a1="0.1" a2="-0.135" b="0" c1="0.615" c2="0.705" c3="0.755" c4="0.085" offsets="0.0 0.0 -1.570796 0.0 0.0 0.0" sign_corrections="1 1 1 -1 1 1"/>
         </robot>)";
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement("robot");
  EXPECT_TRUE(element != nullptr);

  GroupOPWKinematics opw_groups = parseGroupOPWKinematics(g, element, std::array<int, 3>({ 1, 0, 0 }));
  OPWKinematicParameters opw = opw_groups["manipulator"];
  EXPECT_NEAR(opw.a1, 0.1, 1e-8);
  EXPECT_NEAR(opw.a2, -0.135, 1e-8);
  EXPECT_NEAR(opw.b, 0.0, 1e-8);
  EXPECT_NEAR(opw.c1, 0.615, 1e-8);
  EXPECT_NEAR(opw.c2, 0.705, 1e-8);
  EXPECT_NEAR(opw.c3, 0.755, 1e-8);
  EXPECT_NEAR(opw.c4, 0.085, 1e-8);
  EXPECT_NEAR(opw.offsets[0], 0.0, 1e-8);
  EXPECT_NEAR(opw.offsets[1], 0.0, 1e-8);
  EXPECT_NEAR(opw.offsets[2], -1.570796, 1e-8);
  EXPECT_NEAR(opw.offsets[3], 0.0, 1e-8);
  EXPECT_NEAR(opw.offsets[4], 0.0, 1e-8);
  EXPECT_NEAR(opw.offsets[5], 0.0, 1e-8);
  EXPECT_EQ(opw.sign_corrections[0], 1);
  EXPECT_EQ(opw.sign_corrections[1], 1);
  EXPECT_EQ(opw.sign_corrections[2], 1);
  EXPECT_EQ(opw.sign_corrections[3], -1);
  EXPECT_EQ(opw.sign_corrections[4], 1);
  EXPECT_EQ(opw.sign_corrections[5], 1);
}

void printKDLTree(const KDL::SegmentMap::const_iterator& link, const std::string& prefix)
{
  std::cout << prefix << "- Segment " << GetTreeElementSegment(link->second).getName() << " has "
            << GetTreeElementChildren(link->second).size() << " children" << std::endl;

  for (auto child : GetTreeElementChildren(link->second))
    printKDLTree(child, prefix + "  ");
}

tesseract_scene_graph::SceneGraph buildTestSceneGraph()
{
  using namespace tesseract_scene_graph;
  SceneGraph g;

  Link base_link("base_link");
  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");

  g.addLink(std::move(base_link));
  g.addLink(std::move(link_1));
  g.addLink(std::move(link_2));
  g.addLink(std::move(link_3));
  g.addLink(std::move(link_4));
  g.addLink(std::move(link_5));

  Joint base_joint("base_joint");
  base_joint.parent_link_name = "base_link";
  base_joint.child_link_name = "link_1";
  base_joint.type = JointType::FIXED;
  g.addJoint(std::move(base_joint));

  Joint joint_1("joint_1");
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_1));

  Joint joint_2("joint_2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_2";
  joint_2.child_link_name = "link_3";
  joint_2.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_2));

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_3";
  joint_3.child_link_name = "link_4";
  joint_3.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_3));

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4.parent_link_name = "link_2";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;
  g.addJoint(std::move(joint_4));

  return g;
}

TEST(TesseractSceneGraphUnit, LoadKDLUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = buildTestSceneGraph();

  {
    KDL::Tree tree;
    EXPECT_TRUE(parseSceneGraph(g, tree));

    // walk through tree
    std::cout << " ======================================" << std::endl;
    std::cout << " Tree has " << tree.getNrOfSegments() << " link(s) and a root link" << std::endl;
    std::cout << " ======================================" << std::endl;
    auto root = tree.getRootSegment();
    printKDLTree(root, "");

    EXPECT_EQ(tree.getNrOfJoints(), 4);
    EXPECT_EQ(tree.getNrOfSegments(), 5);
  }

  SceneGraph::Ptr g_clone = g.clone();
  {
    KDL::Tree tree;
    EXPECT_TRUE(parseSceneGraph(*g_clone, tree));

    // walk through tree
    std::cout << " ======================================" << std::endl;
    std::cout << " Tree has " << tree.getNrOfSegments() << " link(s) and a root link" << std::endl;
    std::cout << " ======================================" << std::endl;
    auto root = tree.getRootSegment();
    printKDLTree(root, "");

    EXPECT_EQ(tree.getNrOfJoints(), 4);
    EXPECT_EQ(tree.getNrOfSegments(), 5);
  }
}

/// Testing AllowedCollisionMatrix
TEST(TesseractSceneGraphUnit, TestAllowedCollisionMatrix)  // NOLINT
{
  tesseract_scene_graph::AllowedCollisionMatrix acm;

  acm.addAllowedCollision("link1", "link2", "test");
  // collision between link1 and link2 should be allowed
  EXPECT_TRUE(acm.isCollisionAllowed("link1", "link2"));
  // but now between link2 and link3
  EXPECT_FALSE(acm.isCollisionAllowed("link2", "link3"));

  acm.removeAllowedCollision("link1", "link2");
  // now collision link1 and link2 is not allowed anymore
  EXPECT_FALSE(acm.isCollisionAllowed("link1", "link2"));

  acm.addAllowedCollision("link3", "link3", "test");
  EXPECT_EQ(acm.getAllAllowedCollisions().size(), 1);
  acm.clearAllowedCollisions();
  EXPECT_EQ(acm.getAllAllowedCollisions().size(), 0);

  tesseract_scene_graph::AllowedCollisionMatrix acm2;
  acm.addAllowedCollision("link1", "link2", "test");
  acm2.addAllowedCollision("link1", "link2", "test");
  acm2.addAllowedCollision("link1", "link3", "test");
  acm.insertAllowedCollisionMatrix(acm2);

  EXPECT_EQ(acm.getAllAllowedCollisions().size(), 2);
  EXPECT_TRUE(acm.isCollisionAllowed("link1", "link2"));
  EXPECT_TRUE(acm.isCollisionAllowed("link1", "link3"));
  EXPECT_FALSE(acm.isCollisionAllowed("link2", "link3"));
  EXPECT_EQ(acm.getAllAllowedCollisions().size(), 2);
}

TEST(TesseractSceneGraphUnit, TestChangeJointOrigin)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g;

  Link link_1("link_n1");
  Link link_2("link_n2");

  Joint joint_1("joint_n1");
  joint_1.parent_link_name = "link_n1";
  joint_1.child_link_name = "link_n2";
  joint_1.type = JointType::FIXED;

  g.addLink(std::move(link_1));
  g.addLink(std::move(link_2));
  g.addJoint(std::move(joint_1));

  Eigen::Isometry3d new_origin = Eigen::Isometry3d::Identity();
  new_origin.translation()(0) += 1.234;
  g.changeJointOrigin("joint_n1", new_origin);

  // Check that the transform got updated and the edge was recalculated.
  EXPECT_TRUE(g.getJoint("joint_n1")->parent_to_joint_origin_transform.isApprox(new_origin));
  tesseract_scene_graph::SceneGraph::edge_descriptor e = g.getEdge("joint_n1");
  double d = boost::get(boost::edge_weight_t(), g)[e];
  EXPECT_EQ(d, g.getJoint("joint_n1")->parent_to_joint_origin_transform.translation().norm());
}

TEST(TesseractSceneGraphUnit, TesseractSRDFModelUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;

  SRDFModel srdf;

  // Set Name
  srdf.getName() = "test_srdf";
  EXPECT_TRUE(srdf.getName() == "test_srdf");

  // Add chain groups
  auto& chain_groups = srdf.getKinematicsInformation().chain_groups;
  EXPECT_TRUE(chain_groups.empty());

  chain_groups["manipulator_chain"] = { std::make_pair("base_link", "link_5") };
  EXPECT_FALSE(srdf.getKinematicsInformation().chain_groups.empty());

  // Add joint groups
  auto& joint_groups = srdf.getKinematicsInformation().joint_groups;
  EXPECT_TRUE(joint_groups.empty());

  joint_groups["manipulator_joint"] = { "joint_1", "joint_2", "joint_3", "joint_4" };
  EXPECT_FALSE(srdf.getKinematicsInformation().joint_groups.empty());

  // Add link groups
  auto& link_groups = srdf.getKinematicsInformation().link_groups;
  EXPECT_TRUE(link_groups.empty());
  link_groups["manipulator_link"] = { "base_link", "link_1", "link_2", "link_3", "link_4", "link_5" };
  EXPECT_FALSE(srdf.getKinematicsInformation().link_groups.empty());

  // Add group states
  auto& group_state = srdf.getKinematicsInformation().group_states;
  EXPECT_TRUE(group_state.empty());
  tesseract_scene_graph::GroupsJointState joint_state;
  joint_state["joint_1"] = 0;
  joint_state["joint_2"] = 0;
  joint_state["joint_3"] = 0;
  joint_state["joint_4"] = 0;
  group_state["manipulator_chain"]["All Zeros"] = joint_state;
  group_state["manipulator_joint"]["All Zeros"] = joint_state;
  group_state["manipulator_link"]["All Zeros"] = joint_state;
  EXPECT_EQ(srdf.getKinematicsInformation().group_states.size(), 3);

  // Add Tool Center Points
  auto& group_tcps = srdf.getKinematicsInformation().group_tcps;
  EXPECT_TRUE(group_tcps.empty());
  group_tcps["manipulator_chain"]["laser"] = Eigen::Isometry3d::Identity();
  group_tcps["manipulator_joint"]["laser"] = Eigen::Isometry3d::Identity();
  group_tcps["manipulator_link"]["laser"] = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(srdf.getKinematicsInformation().group_tcps.empty());

  // Add disabled collisions
  auto& acm = srdf.getAllowedCollisionMatrix();
  EXPECT_TRUE(acm.getAllAllowedCollisions().empty());
  acm.addAllowedCollision("base_link", "link_1", "Adjacent");
  acm.addAllowedCollision("link_1", "link_2", "Adjacent");
  acm.addAllowedCollision("link_2", "link_3", "Adjacent");
  acm.addAllowedCollision("link_3", "link_4", "Adjacent");
  acm.addAllowedCollision("link_4", "link_5", "Adjacent");
  EXPECT_FALSE(srdf.getAllowedCollisionMatrix().getAllAllowedCollisions().empty());
  srdf.saveToFile(tesseract_common::getTempPath() + "test.srdf");

  SceneGraph g = buildTestSceneGraph();

  tesseract_scene_graph::SRDFModel srdf_reload;
  srdf_reload.initFile(g, tesseract_common::getTempPath() + "test.srdf");
  EXPECT_TRUE(srdf_reload.getName() == "test_srdf");
  EXPECT_FALSE(srdf_reload.getKinematicsInformation().chain_groups.empty());
  EXPECT_FALSE(srdf_reload.getKinematicsInformation().joint_groups.empty());
  EXPECT_FALSE(srdf_reload.getKinematicsInformation().link_groups.empty());
  EXPECT_EQ(srdf_reload.getKinematicsInformation().group_states.size(), 3);
  EXPECT_TRUE(srdf_reload.getKinematicsInformation().group_states["manipulator_chain"].find("All Zeros") !=
              srdf_reload.getKinematicsInformation().group_states["manipulator_chain"].end());
  EXPECT_TRUE(srdf_reload.getKinematicsInformation().group_states["manipulator_joint"].find("All Zeros") !=
              srdf_reload.getKinematicsInformation().group_states["manipulator_joint"].end());
  EXPECT_TRUE(srdf_reload.getKinematicsInformation().group_states["manipulator_link"].find("All Zeros") !=
              srdf_reload.getKinematicsInformation().group_states["manipulator_link"].end());
  EXPECT_FALSE(srdf_reload.getKinematicsInformation().group_tcps.empty());
  EXPECT_TRUE(srdf_reload.getKinematicsInformation().group_tcps["manipulator_chain"].find("laser") !=
              srdf_reload.getKinematicsInformation().group_tcps["manipulator_chain"].end());
  EXPECT_TRUE(srdf_reload.getKinematicsInformation().group_tcps["manipulator_joint"].find("laser") !=
              srdf_reload.getKinematicsInformation().group_tcps["manipulator_joint"].end());
  EXPECT_TRUE(srdf_reload.getKinematicsInformation().group_tcps["manipulator_link"].find("laser") !=
              srdf_reload.getKinematicsInformation().group_tcps["manipulator_link"].end());
  EXPECT_FALSE(srdf_reload.getAllowedCollisionMatrix().getAllAllowedCollisions().empty());
  srdf_reload.saveToFile(tesseract_common::getTempPath() + "test_reload.srdf");
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphInsertEmptyUnit)  // NOLINT
{
  // Test inserting graph into empty graph
  tesseract_scene_graph::SceneGraph g = buildTestSceneGraph();
  tesseract_scene_graph::AllowedCollisionMatrix acm;
  acm.addAllowedCollision("link1", "link2", "test");
  acm.addAllowedCollision("link1", "link3", "test");
  g.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  tesseract_scene_graph::SceneGraph ng;
  EXPECT_TRUE(ng.insertSceneGraph(g));

  // Check Graph
  checkSceneGraph(ng);

  EXPECT_EQ(g.getLinks().size(), ng.getLinks().size());
  EXPECT_EQ(g.getJoints().size(), ng.getJoints().size());
  EXPECT_EQ(g.getRoot(), ng.getRoot());
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  for (const auto& l : g.getLinks())
  {
    EXPECT_TRUE(ng.getLink(l->getName()) != nullptr);
  }

  for (const auto& j : g.getJoints())
  {
    EXPECT_TRUE(ng.getJoint(j->getName()) != nullptr);
  }

  for (const auto& entry : g.getAllowedCollisionMatrix()->getAllAllowedCollisions())
  {
    EXPECT_TRUE(ng.getAllowedCollisionMatrix()->isCollisionAllowed(entry.first.first, entry.first.second));
  }

  // Save Graph
  ng.saveDOT(tesseract_common::getTempPath() + "graph_insert_empty_example.dot");
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphInsertWithoutJointNoPrefixUnit)  // NOLINT
{
  // Test inserting graph into empty graph
  tesseract_scene_graph::SceneGraph g = buildTestSceneGraph();
  tesseract_scene_graph::AllowedCollisionMatrix acm;
  acm.addAllowedCollision("link1", "link2", "test");
  acm.addAllowedCollision("link1", "link3", "test");
  g.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  tesseract_scene_graph::SceneGraph ng = buildTestSceneGraph();
  ng.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  // Insert without prefix which should fail leaving the original graph
  EXPECT_FALSE(ng.insertSceneGraph(g));
  EXPECT_EQ(g.getLinks().size(), ng.getLinks().size());
  EXPECT_EQ(g.getJoints().size(), ng.getJoints().size());
  EXPECT_EQ(g.getRoot(), ng.getRoot());
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  for (const auto& l : g.getLinks())
  {
    EXPECT_TRUE(ng.getLink(l->getName()) != nullptr);
  }

  for (const auto& j : g.getJoints())
  {
    EXPECT_TRUE(ng.getJoint(j->getName()) != nullptr);
  }

  for (const auto& entry : g.getAllowedCollisionMatrix()->getAllAllowedCollisions())
  {
    EXPECT_TRUE(ng.getAllowedCollisionMatrix()->isCollisionAllowed(entry.first.first, entry.first.second));
  }
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphInsertWithoutJointWithPrefixUnit)  // NOLINT
{
  // Test inserting graph into empty graph
  tesseract_scene_graph::SceneGraph g = buildTestSceneGraph();
  tesseract_scene_graph::AllowedCollisionMatrix acm;
  acm.addAllowedCollision("link1", "link2", "test");
  acm.addAllowedCollision("link1", "link3", "test");
  g.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  tesseract_scene_graph::SceneGraph ng = buildTestSceneGraph();
  ng.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  std::string prefix = "r1::";
  EXPECT_TRUE(ng.insertSceneGraph(g, prefix));
  EXPECT_FALSE(ng.isTree());

  // Check Graph
  checkSceneGraph(ng);

  EXPECT_EQ(2 * g.getLinks().size(), ng.getLinks().size());
  EXPECT_EQ(2 * g.getJoints().size(), ng.getJoints().size());
  EXPECT_EQ(g.getRoot(), ng.getRoot());
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 4);

  for (const auto& l : g.getLinks())
  {
    EXPECT_TRUE(ng.getLink(l->getName()) != nullptr);
    EXPECT_TRUE(ng.getLink(prefix + l->getName()) != nullptr);
  }

  for (const auto& j : g.getJoints())
  {
    EXPECT_TRUE(ng.getJoint(j->getName()) != nullptr);
    EXPECT_TRUE(ng.getJoint(prefix + j->getName()) != nullptr);
  }

  for (const auto& entry : g.getAllowedCollisionMatrix()->getAllAllowedCollisions())
  {
    EXPECT_TRUE(ng.getAllowedCollisionMatrix()->isCollisionAllowed(entry.first.first, entry.first.second));
    EXPECT_TRUE(
        ng.getAllowedCollisionMatrix()->isCollisionAllowed(prefix + entry.first.first, prefix + entry.first.second));
  }

  // Save Graph
  ng.saveDOT(tesseract_common::getTempPath() + "graph_insert_example.dot");
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphInsertWithJointWithPrefixUnit)  // NOLINT
{
  // Test inserting graph into empty graph
  tesseract_scene_graph::SceneGraph g = buildTestSceneGraph();
  tesseract_scene_graph::AllowedCollisionMatrix acm;
  acm.addAllowedCollision("link1", "link2", "test");
  acm.addAllowedCollision("link1", "link3", "test");
  g.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  tesseract_scene_graph::SceneGraph ng = buildTestSceneGraph();
  ng.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  std::string prefix = "r1::";

  tesseract_scene_graph::Joint new_joint("insert_graph_joint");
  new_joint.parent_link_name = "base_link";
  new_joint.child_link_name = prefix + new_joint.parent_link_name;
  new_joint.type = tesseract_scene_graph::JointType::FIXED;
  new_joint.parent_to_joint_origin_transform = Eigen::Translation3d(1, 0, 0) * Eigen::Isometry3d::Identity();
  EXPECT_TRUE(ng.insertSceneGraph(g, std::move(new_joint), prefix));
  EXPECT_TRUE(ng.isTree());

  // Check Graph
  checkSceneGraph(ng);

  EXPECT_EQ(2 * g.getLinks().size(), ng.getLinks().size());
  EXPECT_EQ((2 * g.getJoints().size()) + 1, ng.getJoints().size());
  EXPECT_EQ(g.getRoot(), ng.getRoot());
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 4);

  for (const auto& l : g.getLinks())
  {
    EXPECT_TRUE(ng.getLink(l->getName()) != nullptr);
    EXPECT_TRUE(ng.getLink(prefix + l->getName()) != nullptr);
  }

  for (const auto& j : g.getJoints())
  {
    EXPECT_TRUE(ng.getJoint(j->getName()) != nullptr);
    EXPECT_TRUE(ng.getJoint(prefix + j->getName()) != nullptr);
  }

  for (const auto& entry : g.getAllowedCollisionMatrix()->getAllAllowedCollisions())
  {
    EXPECT_TRUE(ng.getAllowedCollisionMatrix()->isCollisionAllowed(entry.first.first, entry.first.second));
    EXPECT_TRUE(
        ng.getAllowedCollisionMatrix()->isCollisionAllowed(prefix + entry.first.first, prefix + entry.first.second));
  }

  // Save Graph
  ng.saveDOT(tesseract_common::getTempPath() + "graph_insert_with_joint_example.dot");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
