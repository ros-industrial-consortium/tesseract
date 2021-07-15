#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_common/utils.h>

using namespace tesseract_scene_graph;

std::string toString(const SceneGraph::Path& path)
{
  std::stringstream ss;
  ss << path;
  return ss.str();
}

std::string toString(bool b) { return b ? "true" : "false"; }

int main(int /*argc*/, char** /*argv*/)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  // documentation:start:1: Create scene graph
  SceneGraph g;
  // documentation:end:1: Create scene graph

  // documentation:start:2: Create links
  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");
  // documentation:end:2: Create links

  // documentation:start:3: Add links
  g.addLink(link_1);
  g.addLink(link_2);
  g.addLink(link_3);
  g.addLink(link_4);
  g.addLink(link_5);
  // documentation:end:3: Add links

  // documentation:start:4: Create joints
  Joint joint_1("joint_1");
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::FIXED;

  Joint joint_2("joint_2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_2";
  joint_2.child_link_name = "link_3";
  joint_2.type = JointType::PLANAR;

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_3";
  joint_3.child_link_name = "link_4";
  joint_3.type = JointType::FLOATING;

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4.parent_link_name = "link_2";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;
  // documentation:end:4: Create joints

  // documentation:start:5: Add joints
  g.addJoint(joint_1);
  g.addJoint(joint_2);
  g.addJoint(joint_3);
  g.addJoint(joint_4);
  // documentation:end:5: Add joints

  // documentation:start:6: Check getAdjacentLinkNames Method
  std::vector<std::string> adjacent_links = g.getAdjacentLinkNames("link_3");
  for (const auto& adj : adjacent_links)
    CONSOLE_BRIDGE_logInform(adj.c_str());
  // documentation:end:6: Check getAdjacentLinkNames Method

  // documentation:start:7: Check getInvAdjacentLinkNames Method
  std::vector<std::string> inv_adjacent_links = g.getInvAdjacentLinkNames("link_3");
  for (const auto& inv_adj : inv_adjacent_links)
    CONSOLE_BRIDGE_logInform(inv_adj.c_str());
  // documentation:end:7: Check getInvAdjacentLinkNames Method

  // documentation:start:8: Check getLinkChildrenNames
  std::vector<std::string> child_link_names = g.getLinkChildrenNames("link_2");
  for (const auto& child_link : child_link_names)
    CONSOLE_BRIDGE_logInform(child_link.c_str());
  // documentation:end:8: Check getLinkChildrenNames

  // documentation:start:9: Check getJointChildrenNames
  child_link_names = g.getJointChildrenNames("joint_1");
  for (const auto& child_link : child_link_names)
    CONSOLE_BRIDGE_logInform(child_link.c_str());
  // documentation:end:9: Check getJointChildrenNames

  // documentation:start:10: Save Graph
  g.saveDOT(tesseract_common::getTempPath() + "graph_acyclic_tree_example.dot");
  // documentation:end:10: Save Graph

  // documentation:start:11: Test if the graph is Acyclic
  bool is_acyclic = g.isAcyclic();
  CONSOLE_BRIDGE_logInform(toString(is_acyclic).c_str());
  // documentation:end:11: Test if the graph is Acyclic

  // documentation:start:12: Test if the graph is Tree
  bool is_tree = g.isTree();
  CONSOLE_BRIDGE_logInform(toString(is_tree).c_str());
  // documentation:end:12: Test if the graph is Tree

  // documentation:start:13: Test for unused links
  Link link_6("link_6");
  g.addLink(link_6);
  is_tree = g.isTree();
  CONSOLE_BRIDGE_logInform(toString(is_tree).c_str());
  // documentation:end:13: Test for unused links

  // documentation:start:14: Remove unused link
  g.removeLink("link_6");
  is_tree = g.isTree();
  CONSOLE_BRIDGE_logInform(toString(is_tree).c_str());
  // documentation:end:14: Remove unused link

  // documentation:start:15: Add new joint
  Joint joint_5("joint_5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_5";
  joint_5.child_link_name = "link_4";
  joint_5.type = JointType::CONTINUOUS;
  g.addJoint(joint_5);
  // documentation:end:15: Add new joint

  // documentation:start:16: Save new graph
  g.saveDOT(tesseract_common::getTempPath() + "graph_acyclic_not_tree_example.dot");
  // documentation:end:16: Save new graph

  // documentation:start:17: Test again if the graph is Acyclic
  is_acyclic = g.isAcyclic();
  CONSOLE_BRIDGE_logInform(toString(is_acyclic).c_str());
  // documentation:end:17: Test again if the graph is Acyclic

  // documentation:start:18: Test again if the graph is Tree
  is_tree = g.isTree();
  CONSOLE_BRIDGE_logInform(toString(is_tree).c_str());
  // documentation:end:18: Test again if the graph is Tree

  // documentation:start:19: Get Shortest Path
  SceneGraph::Path path = g.getShortestPath("link_1", "link_4");
  CONSOLE_BRIDGE_logInform(toString(path).c_str());
  // documentation:end:19: Get Shortest Path
}
