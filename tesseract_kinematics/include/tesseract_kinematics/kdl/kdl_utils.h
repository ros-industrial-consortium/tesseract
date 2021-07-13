/**
 * @file kdl_utils.h
 * @brief Tesseract KDL utility functions.
 *
 * @author Levi Armstrong
 * @date May 27, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_KDL_UTILS_H
#define TESSERACT_KINEMATICS_KDL_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Eigen>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/parser/kdl_parser.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/core/types.h>

namespace tesseract_kinematics
{
/**
 * @brief Convert KDL::Frame to Eigen::Isometry3d
 * @param frame Input KDL Frame
 * @param transform Output Eigen transform (Isometry3d)
 */
inline void KDLToEigen(const KDL::Frame& frame, Eigen::Isometry3d& transform)
{
  transform.setIdentity();

  // translation
  for (int i = 0; i < 3; ++i)
    transform(i, 3) = frame.p[i];

  // rotation matrix
  for (int i = 0; i < 9; ++i)
    transform(i / 3, i % 3) = frame.M.data[i];
}

/**
 * @brief Convert vector of KDL::Frame to vector Eigen::Isometry3d
 * @param frames Input KDL Frames
 * @param transforms Output Eigen transforms (Isometry3d)
 */
inline void KDLToEigen(const std::vector<KDL::Frame>& frames, tesseract_common::VectorIsometry3d& transforms)
{
  transforms.resize(frames.size());
  for (size_t i = 0; i < frames.size(); ++i)
  {
    KDLToEigen(frames[i], transforms[i]);
  }
}

/**
 * @brief Convert Eigen::Isometry3d to KDL::Frame
 * @param transform Input Eigen transform (Isometry3d)
 * @param frame Output KDL Frame
 */
inline void EigenToKDL(const Eigen::Isometry3d& transform, KDL::Frame& frame)
{
  frame.Identity();

  for (int i = 0; i < 3; ++i)
    frame.p[i] = transform(i, 3);

  for (int i = 0; i < 9; ++i)
    frame.M.data[i] = transform(i / 3, i % 3);
}

/**
 * @brief Convert KDL::Jacobian to Eigen::Matrix
 * @param jacobian Input KDL Jacobian
 * @param matrix Output Eigen MatrixXd
 */
inline void KDLToEigen(const KDL::Jacobian& jacobian, Eigen::Ref<Eigen::MatrixXd> matrix)
{
  assert(matrix.rows() == jacobian.rows());
  assert(matrix.cols() == jacobian.columns());

  for (unsigned i = 0; i < jacobian.rows(); ++i)
    for (unsigned j = 0; j < jacobian.columns(); ++j)
      matrix(i, j) = jacobian(i, j);
}

/**
 * @brief Convert a subset of KDL::Jacobian to Eigen::Matrix
 * @param jacobian Input KDL Jacobian
 * @param q_nrs Input the columns to use
 * @param matrix Output Eigen MatrixXd
 */
inline void KDLToEigen(const KDL::Jacobian& jacobian, const std::vector<int>& q_nrs, Eigen::Ref<Eigen::MatrixXd> matrix)
{
  assert(matrix.rows() == jacobian.rows());
  assert(static_cast<unsigned>(matrix.cols()) == q_nrs.size());

  for (int i = 0; i < static_cast<int>(jacobian.rows()); ++i)
    for (int j = 0; j < static_cast<int>(q_nrs.size()); ++j)
      matrix(i, j) = jacobian(static_cast<unsigned>(i), static_cast<unsigned>(q_nrs[static_cast<size_t>(j)]));
}

/**
 * @brief Convert Eigen::Vector to KDL::JntArray
 * @param vec Input Eigen vector
 * @param joints Output KDL joint array
 */
inline void EigenToKDL(const Eigen::Ref<const Eigen::VectorXd>& vec, KDL::JntArray& joints) { joints.data = vec; }

/**
 * @brief Convert KDL::JntArray to Eigen::Vector
 * @param joints Input KDL joint array
 * @param vec Output Eigen vector
 */
inline void KDLToEigen(const KDL::JntArray& joints, Eigen::Ref<Eigen::VectorXd> vec) { vec = joints.data; }

/**
 * @brief The KDLChainData struct
 *
 * This contains common data extracted when parsing
 * a kdl chain from the scene graph
 */
struct KDLChainData
{
  KDL::Chain robot_chain;                   /**< KDL Chain object */
  KDL::Tree kdl_tree;                       /**< KDL tree object */
  SynchronizableData data;                  /**< Synchronizable data */
  std::map<std::string, int> segment_index; /**< A map from chain link name to kdl chain segment number */
  std::vector<std::pair<std::string, std::string>> chains; /**< The chains used to create the object */
};

/**
 * @brief Parse KDL chain data from the scene graph
 * @param results KDL Chain data
 * @param scene_graph The Scene Graph
 * @param chains A vector of link pairs that represent a chain which all get concatenated together. The firts link
 *        should be the base link and the second link should the tip link of the chain.
 * @return True if successful otherwise false
 */
inline bool parseSceneGraph(KDLChainData& results,
                            const tesseract_scene_graph::SceneGraph& scene_graph,
                            const std::vector<std::pair<std::string, std::string>>& chains)
{
  if (!tesseract_scene_graph::parseSceneGraph(scene_graph, results.kdl_tree))
  {
    CONSOLE_BRIDGE_logError("Failed to parse KDL tree from Scene Graph");
    return false;
  }

  results.chains = chains;
  results.data.base_link_name = chains.front().first;
  for (const auto& chain : chains)
  {
    KDL::Chain sub_chain;
    if (!results.kdl_tree.getChain(chain.first, chain.second, sub_chain))
    {
      CONSOLE_BRIDGE_logError(
          "Failed to initialize KDL between links: '%s' and '%s'", chain.first.c_str(), chain.second.c_str());
      return false;
    }
    results.robot_chain.addChain(sub_chain);
  }
  results.data.tip_link_name = chains.back().second;

  results.data.joint_names.resize(results.robot_chain.getNrOfJoints());
  results.data.limits.joint_limits.resize(results.robot_chain.getNrOfJoints(), 2);
  results.data.limits.velocity_limits.resize(results.robot_chain.getNrOfJoints());
  results.data.limits.acceleration_limits.resize(results.robot_chain.getNrOfJoints());

  results.segment_index[results.data.base_link_name] = 0;
  results.data.link_names.push_back(results.data.base_link_name);
  results.data.active_link_names.clear();
  bool found = false;
  for (unsigned i = 0, j = 0; i < results.robot_chain.getNrOfSegments(); ++i)
  {
    const KDL::Segment& seg = results.robot_chain.getSegment(i);
    const KDL::Joint& jnt = seg.getJoint();
    results.data.link_names.push_back(seg.getName());

    // KDL segments does not contain the the base link in this list. When calling function that take segmentNr, like
    // JntToCart to get the base link transform you would pass an index of zero and for subsequent links it is
    // index + 1. This was determined through testing which is captured in this packages unit tests.
    results.segment_index[seg.getName()] = static_cast<int>(i + 1);

    if (found)
      results.data.active_link_names.push_back(seg.getName());

    if (jnt.getType() == KDL::Joint::None)
      continue;

    if (!found)
    {
      found = true;
      results.data.active_link_names.push_back(seg.getName());
    }

    results.data.joint_names[j] = jnt.getName();
    const tesseract_scene_graph::Joint::ConstPtr& joint = scene_graph.getJoint(jnt.getName());
    results.data.limits.joint_limits(j, 0) = joint->limits->lower;
    results.data.limits.joint_limits(j, 1) = joint->limits->upper;
    results.data.limits.velocity_limits(j) = joint->limits->velocity;
    results.data.limits.acceleration_limits(j) = joint->limits->acceleration;

    // Need to set limits for continuous joints. TODO: This may not be required
    // by the optimization library but may be nice to have
    if (joint->type == tesseract_scene_graph::JointType::CONTINUOUS &&
        tesseract_common::almostEqualRelativeAndAbs(
            results.data.limits.joint_limits(j, 0), results.data.limits.joint_limits(j, 1), 1e-5))
    {
      results.data.limits.joint_limits(j, 0) = -4 * M_PI;
      results.data.limits.joint_limits(j, 1) = +4 * M_PI;
    }
    ++j;
  }

  results.data.redundancy_indices.clear();
  for (std::size_t i = 0; i < results.data.joint_names.size(); ++i)
  {
    const auto& joint = scene_graph.getJoint(results.data.joint_names[i]);
    switch (joint->type)
    {
      case tesseract_scene_graph::JointType::REVOLUTE:
      case tesseract_scene_graph::JointType::CONTINUOUS:
        results.data.redundancy_indices.push_back(static_cast<Eigen::Index>(i));
        break;
      default:
        break;
    }
  }

  return true;
}

/**
 * @brief Parse KDL chain data from the scene graph
 * @param results KDL Chain data
 * @param scene_graph The Scene Graph
 * @param base_name The base link name of chain
 * @param tip_name The tip link name of chain
 * @return True if successful otherwise false
 */
inline bool parseSceneGraph(KDLChainData& results,
                            const tesseract_scene_graph::SceneGraph& scene_graph,
                            const std::string& base_name,
                            const std::string& tip_name)
{
  std::vector<std::pair<std::string, std::string>> chains;
  chains.emplace_back(base_name, tip_name);
  return parseSceneGraph(results, scene_graph, chains);
}
}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_KDL_UTILS_H
