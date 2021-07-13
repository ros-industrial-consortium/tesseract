/**
 * @file manipulator_manager.h
 * @brief This managers everything about all manipulator, like forward kinematics, inverse kinematics, tcp's, etc.
 *
 * @author Levi Armstrong
 * @date Sep 7, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_environment/core/manipulator_manager.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain_factory.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr_factory.h>
#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>
#include <tesseract_kinematics/core/rep_inverse_kinematics.h>

namespace tesseract_environment
{
bool ManipulatorManager::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                              tesseract_srdf::KinematicsInformation kinematics_information)
{
  if (scene_graph == nullptr)
    return false;

  scene_graph_ = std::move(scene_graph);
  kinematics_information_.clear();

  fwd_kin_chain_default_factory_ = std::make_shared<tesseract_kinematics::KDLFwdKinChainFactory>();
  registerFwdKinematicsFactory(fwd_kin_chain_default_factory_);

  fwd_kin_tree_default_factory_ = std::make_shared<tesseract_kinematics::KDLFwdKinTreeFactory>();
  registerFwdKinematicsFactory(fwd_kin_tree_default_factory_);

  inv_kin_chain_default_factory_ = std::make_shared<tesseract_kinematics::KDLInvKinChainLMAFactory>();
  registerInvKinematicsFactory(inv_kin_chain_default_factory_);

  initialized_ = addKinematicsInformation(kinematics_information);
  return initialized_;
}

bool ManipulatorManager::isInitialized() const { return initialized_; }

void ManipulatorManager::onEnvironmentChanged(const Commands& commands)
{
  for (auto it = commands.begin() + revision_; it != commands.end(); ++it)
  {
    const Command::ConstPtr& command = *it;
    if (!command)
      throw std::runtime_error("ManipulatorManager: Commands constains nullptr's");

    switch (command->getType())
    {
      case CommandType::CHANGE_JOINT_POSITION_LIMITS:
      {
        const auto& cmd = static_cast<const tesseract_environment::ChangeJointPositionLimitsCommand&>(*command);
        const std::unordered_map<std::string, std::pair<double, double>>& cmd_limits = cmd.getLimits();
        for (auto& fk : fwd_kin_manipulators_)
        {
          tesseract_common::KinematicLimits limits = fk.second->getLimits();
          const std::vector<std::string>& joint_names = fk.second->getJointNames();
          bool changed = false;
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {
            auto it = cmd_limits.find(joint_names[i]);
            if (it != cmd_limits.end())
            {
              limits.joint_limits(static_cast<Eigen::Index>(i), 0) = it->second.first;
              limits.joint_limits(static_cast<Eigen::Index>(i), 1) = it->second.second;
              changed = true;
            }
          }
          if (changed)
            fk.second->setLimits(limits);
        }

        for (auto& ik : inv_kin_manipulators_)
        {
          tesseract_common::KinematicLimits limits = ik.second->getLimits();
          const std::vector<std::string>& joint_names = ik.second->getJointNames();
          bool changed = false;
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {
            auto it = cmd_limits.find(joint_names[i]);
            if (it != cmd_limits.end())
            {
              limits.joint_limits(static_cast<Eigen::Index>(i), 0) = it->second.first;
              limits.joint_limits(static_cast<Eigen::Index>(i), 1) = it->second.second;
              changed = true;
            }
          }
          if (changed)
            ik.second->setLimits(limits);
        }
        break;
      }
      case CommandType::CHANGE_JOINT_VELOCITY_LIMITS:
      {
        const auto& cmd = static_cast<const tesseract_environment::ChangeJointVelocityLimitsCommand&>(*command);
        const std::unordered_map<std::string, double>& cmd_limits = cmd.getLimits();
        for (auto& fk : fwd_kin_manipulators_)
        {
          tesseract_common::KinematicLimits limits = fk.second->getLimits();
          const std::vector<std::string>& joint_names = fk.second->getJointNames();
          bool changed = false;
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {
            auto it = cmd_limits.find(joint_names[i]);
            if (it != cmd_limits.end())
            {
              limits.velocity_limits(static_cast<Eigen::Index>(i)) = it->second;
              changed = true;
            }
          }
          if (changed)
            fk.second->setLimits(limits);
        }

        for (auto& ik : inv_kin_manipulators_)
        {
          tesseract_common::KinematicLimits limits = ik.second->getLimits();
          const std::vector<std::string>& joint_names = ik.second->getJointNames();
          bool changed = false;
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {
            auto it = cmd_limits.find(joint_names[i]);
            if (it != cmd_limits.end())
            {
              limits.velocity_limits(static_cast<Eigen::Index>(i)) = it->second;
              changed = true;
            }
          }
          if (changed)
            ik.second->setLimits(limits);
        }
        break;
      }
      case CommandType::CHANGE_JOINT_ACCELERATION_LIMITS:
      {
        const auto& cmd = static_cast<const tesseract_environment::ChangeJointAccelerationLimitsCommand&>(*command);
        const std::unordered_map<std::string, double>& cmd_limits = cmd.getLimits();
        for (auto& fk : fwd_kin_manipulators_)
        {
          tesseract_common::KinematicLimits limits = fk.second->getLimits();
          const std::vector<std::string>& joint_names = fk.second->getJointNames();
          bool changed = false;
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {
            auto it = cmd_limits.find(joint_names[i]);
            if (it != cmd_limits.end())
            {
              limits.acceleration_limits(static_cast<Eigen::Index>(i)) = it->second;
              changed = true;
            }
          }
          if (changed)
            fk.second->setLimits(limits);
        }

        for (auto& ik : inv_kin_manipulators_)
        {
          tesseract_common::KinematicLimits limits = ik.second->getLimits();
          const std::vector<std::string>& joint_names = ik.second->getJointNames();
          bool changed = false;
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {
            auto it = cmd_limits.find(joint_names[i]);
            if (it != cmd_limits.end())
            {
              limits.acceleration_limits(static_cast<Eigen::Index>(i)) = it->second;
              changed = true;
            }
          }
          if (changed)
            ik.second->setLimits(limits);
        }
        break;
      }
      case CommandType::ADD_KINEMATICS_INFORMATION:
      {
        const auto& cmd = static_cast<const tesseract_environment::AddKinematicsInformationCommand&>(*command);
        addKinematicsInformation(cmd.getKinematicsInformation());
        break;
      }
      case CommandType::CHANGE_LINK_VISIBILITY:
      case CommandType::CHANGE_LINK_COLLISION_ENABLED:
      case CommandType::ADD_ALLOWED_COLLISION:
      case CommandType::REMOVE_ALLOWED_COLLISION_LINK:
      case CommandType::CHANGE_COLLISION_MARGINS:
      {
        break;
      }
      default:
      {
        for (auto& fk : fwd_kin_manipulators_)
          fk.second->update();

        for (auto& ik : inv_kin_manipulators_)
          ik.second->update();
        break;
      }
    }
  }

  revision_ = static_cast<int>(commands.size());
}

ManipulatorManager::Ptr ManipulatorManager::clone(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph) const
{
  auto cloned_manager = std::make_shared<ManipulatorManager>(*this);
  cloned_manager->scene_graph_ = std::move(scene_graph);
  return cloned_manager;
}

bool ManipulatorManager::addKinematicsInformation(const tesseract_srdf::KinematicsInformation& kinematics_information)
{
  bool success = true;

  tesseract_srdf::GroupNames& gn = kinematics_information_.group_names;
  const tesseract_srdf::GroupNames& cgn = kinematics_information.group_names;
  gn.insert(gn.end(), cgn.begin(), cgn.end());

  tesseract_srdf::ChainGroups& cg = kinematics_information_.chain_groups;
  const tesseract_srdf::ChainGroups& ccg = kinematics_information.chain_groups;
  cg.insert(ccg.begin(), ccg.end());

  tesseract_srdf::JointGroups& jg = kinematics_information_.joint_groups;
  const tesseract_srdf::JointGroups& cjg = kinematics_information.joint_groups;
  jg.insert(cjg.begin(), cjg.end());

  tesseract_srdf::LinkGroups& lg = kinematics_information_.link_groups;
  const tesseract_srdf::LinkGroups& clg = kinematics_information.link_groups;
  lg.insert(clg.begin(), clg.end());

  tesseract_srdf::GroupTCPs& gtcps = kinematics_information_.group_tcps;
  const tesseract_srdf::GroupTCPs& cgtcps = kinematics_information.group_tcps;
  for (const auto& t : cgtcps)
  {
    auto it = gtcps.find(t.first);
    if (it == gtcps.end())
      gtcps[t.first] = t.second;
    else
      it->second.insert(t.second.begin(), t.second.end());
  }

  tesseract_srdf::GroupJointStates& gjs = kinematics_information_.group_states;
  const tesseract_srdf::GroupJointStates& cgjs = kinematics_information.group_states;
  for (const auto& t : cgjs)
  {
    auto it = gjs.find(t.first);
    if (it == gjs.end())
      gjs[t.first] = t.second;
    else
      it->second.insert(t.second.begin(), t.second.end());
  }

  tesseract_srdf::GroupOPWKinematics& gopwk = kinematics_information_.group_opw_kinematics;
  const tesseract_srdf::GroupOPWKinematics& cgopwk = kinematics_information.group_opw_kinematics;
  gopwk.insert(cgopwk.begin(), cgopwk.end());

  tesseract_srdf::GroupROPKinematics& gropk = kinematics_information_.group_rop_kinematics;
  const tesseract_srdf::GroupROPKinematics& cgropk = kinematics_information.group_rop_kinematics;
  gropk.insert(cgropk.begin(), cgropk.end());

  tesseract_srdf::GroupREPKinematics& grepk = kinematics_information_.group_rep_kinematics;
  const tesseract_srdf::GroupREPKinematics& cgrepk = kinematics_information.group_rep_kinematics;
  grepk.insert(cgrepk.begin(), cgrepk.end());

  for (const auto& group : kinematics_information.chain_groups)
    success &= registerDefaultChainSolver(group.first, group.second);

  for (const auto& group : kinematics_information.joint_groups)
    success &= registerDefaultJointSolver(group.first, group.second);

  //  for (const auto& group : kinematics_information_.link_groups)
  //    success &= registerDefaultLinkSolver(group.first, group.second);

  for (const auto& group : kinematics_information.group_opw_kinematics)
    success &= registerOPWSolver(group.first, group.second);

  for (const auto& group : kinematics_information.group_rop_kinematics)
    success &= registerROPSolver(group.first, group.second);

  for (const auto& group : kinematics_information.group_rep_kinematics)
    success &= registerREPSolver(group.first, group.second);

  return success;
}

const tesseract_srdf::KinematicsInformation& ManipulatorManager::getKinematicsInformation() const
{
  return kinematics_information_;
}

const tesseract_srdf::GroupNames& ManipulatorManager::getGroupNames() const
{
  return kinematics_information_.group_names;
}

bool ManipulatorManager::hasGroup(const std::string& group_name) const
{
  const tesseract_srdf::GroupNames& group_names = kinematics_information_.group_names;
  return std::find(group_names.begin(), group_names.end(), group_name) != group_names.end();
}

bool ManipulatorManager::addChainGroup(const std::string& group_name, const tesseract_srdf::ChainGroup& chain_group)
{
  if (hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Group name is already taken!");
    return false;
  }

  if (!registerDefaultChainSolver(group_name, chain_group))
    return false;

  kinematics_information_.chain_groups[group_name] = chain_group;
  kinematics_information_.group_names.push_back(group_name);
  return true;
}

void ManipulatorManager::removeChainGroup(const std::string& group_name)
{
  if (kinematics_information_.chain_groups.erase(group_name) > 0)
  {
    tesseract_srdf::GroupNames& group_names = kinematics_information_.group_names;
    group_names.erase(std::remove_if(
        group_names.begin(), group_names.end(), [group_name](const std::string& gn) { return gn == group_name; }));
    removeFwdKinematicSolver(group_name);
    removeInvKinematicSolver(group_name);
  }
}

const tesseract_srdf::ChainGroup& ManipulatorManager::getChainGroup(const std::string& group_name) const
{
  return kinematics_information_.chain_groups.at(group_name);
}

bool ManipulatorManager::hasChainGroup(const std::string& group_name) const
{
  return (kinematics_information_.chain_groups.find(group_name) != kinematics_information_.chain_groups.end());
}

const tesseract_srdf::ChainGroups& ManipulatorManager::getChainGroups() const
{
  return kinematics_information_.chain_groups;
}

bool ManipulatorManager::addJointGroup(const std::string& group_name, const tesseract_srdf::JointGroup& joint_group)
{
  if (hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Group name is already taken!");
    return false;
  }

  if (!registerDefaultJointSolver(group_name, joint_group))
    return false;

  kinematics_information_.joint_groups[group_name] = joint_group;
  kinematics_information_.group_names.push_back(group_name);
  return true;
}

void ManipulatorManager::removeJointGroup(const std::string& group_name)
{
  if (kinematics_information_.joint_groups.erase(group_name) > 0)
  {
    tesseract_srdf::GroupNames& group_names = kinematics_information_.group_names;
    group_names.erase(std::remove_if(
        group_names.begin(), group_names.end(), [group_name](const std::string& gn) { return gn == group_name; }));
    removeFwdKinematicSolver(group_name);
    removeInvKinematicSolver(group_name);
  }
}

bool ManipulatorManager::hasJointGroup(const std::string& group_name) const
{
  return (kinematics_information_.joint_groups.find(group_name) != kinematics_information_.joint_groups.end());
}

const tesseract_srdf::JointGroup& ManipulatorManager::getJointGroup(const std::string& group_name) const
{
  return kinematics_information_.joint_groups.at(group_name);
}

const tesseract_srdf::JointGroups& ManipulatorManager::getJointGroups() const
{
  return kinematics_information_.joint_groups;
}

bool ManipulatorManager::addLinkGroup(const std::string& group_name, const tesseract_srdf::LinkGroup& link_group)
{
  if (hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Group name is already taken!");
    return false;
  }

  if (!registerDefaultLinkSolver(group_name, link_group))
    return false;

  kinematics_information_.link_groups[group_name] = link_group;
  kinematics_information_.group_names.push_back(group_name);
  return true;
}

void ManipulatorManager::removeLinkGroup(const std::string& group_name)
{
  if (kinematics_information_.link_groups.erase(group_name) > 0)
  {
    tesseract_srdf::GroupNames& group_names = kinematics_information_.group_names;
    group_names.erase(std::remove_if(
        group_names.begin(), group_names.end(), [group_name](const std::string& gn) { return gn == group_name; }));
    removeFwdKinematicSolver(group_name);
    removeInvKinematicSolver(group_name);
  }
}

bool ManipulatorManager::hasLinkGroup(const std::string& group_name) const
{
  return (kinematics_information_.link_groups.find(group_name) != kinematics_information_.link_groups.end());
}

const tesseract_srdf::LinkGroup& ManipulatorManager::getLinkGroup(const std::string& group_name) const
{
  return kinematics_information_.link_groups.at(group_name);
}

const tesseract_srdf::LinkGroups& ManipulatorManager::getLinkGroups() const
{
  return kinematics_information_.link_groups;
}

bool ManipulatorManager::addROPKinematicsSolver(const std::string& group_name,
                                                const tesseract_srdf::ROPKinematicParameters& rop_group)
{
  if (!hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Group %s does not exist!", group_name.c_str());
    return false;
  }

  if (!registerROPSolver(group_name, rop_group))
    return false;

  kinematics_information_.group_rop_kinematics[group_name] = rop_group;
  return true;
}

void ManipulatorManager::removeROPKinematicsSolver(const std::string& group_name)
{
  if (kinematics_information_.group_rop_kinematics.erase(group_name) > 0)
    removeInvKinematicSolver(group_name, "RobotOnPositionerInvKin");
}

bool ManipulatorManager::hasROPKinematicsSolver(const std::string& group_name) const
{
  return (kinematics_information_.group_rop_kinematics.find(group_name) !=
          kinematics_information_.group_rop_kinematics.end());
}

const tesseract_srdf::ROPKinematicParameters&
ManipulatorManager::getROPKinematicsSolver(const std::string& group_name) const
{
  return kinematics_information_.group_rop_kinematics.at(group_name);
}

const tesseract_srdf::GroupROPKinematics& ManipulatorManager::getROPKinematicsSolvers() const
{
  return kinematics_information_.group_rop_kinematics;
}

bool ManipulatorManager::addREPKinematicsSolver(const std::string& group_name,
                                                const tesseract_srdf::REPKinematicParameters& rep_group)
{
  if (!hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Group %s does not exist!", group_name.c_str());
    return false;
  }

  if (!registerREPSolver(group_name, rep_group))
    return false;

  kinematics_information_.group_rep_kinematics[group_name] = rep_group;
  return true;
}

void ManipulatorManager::removeREPKinematicsSolver(const std::string& group_name)
{
  if (kinematics_information_.group_rep_kinematics.erase(group_name) > 0)
    removeInvKinematicSolver(group_name, "RobotWithExternalPositionerInvKin");
}

bool ManipulatorManager::hasREPKinematicsSolver(const std::string& group_name) const
{
  return (kinematics_information_.group_rep_kinematics.find(group_name) !=
          kinematics_information_.group_rep_kinematics.end());
}

const tesseract_srdf::REPKinematicParameters&
ManipulatorManager::getREPKinematicsSolver(const std::string& group_name) const
{
  return kinematics_information_.group_rep_kinematics.at(group_name);
}

const tesseract_srdf::GroupREPKinematics& ManipulatorManager::getREPKinematicsSolvers() const
{
  return kinematics_information_.group_rep_kinematics;
}

bool ManipulatorManager::addOPWKinematicsSolver(const std::string& group_name,
                                                const tesseract_srdf::OPWKinematicParameters& opw_params)
{
  if (!hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Tried to add OPW Kinematics Solver for group that does not exist!");
    return false;
  }

  if (!registerOPWSolver(group_name, opw_params))
    return false;

  kinematics_information_.group_opw_kinematics[group_name] = opw_params;
  return true;
}

void ManipulatorManager::removeOPWKinematicsSovler(const std::string& group_name)
{
  if (kinematics_information_.group_opw_kinematics.erase(group_name) > 0)
    removeInvKinematicSolver(group_name, "OPWInvKin");
}

bool ManipulatorManager::hasOPWKinematicsSolver(const std::string& group_name) const
{
  return (kinematics_information_.group_opw_kinematics.find(group_name) !=
          kinematics_information_.group_opw_kinematics.end());
}

const tesseract_srdf::OPWKinematicParameters&
ManipulatorManager::getOPWKinematicsSolver(const std::string& group_name) const
{
  return kinematics_information_.group_opw_kinematics.at(group_name);
}

const tesseract_srdf::GroupOPWKinematics& ManipulatorManager::getOPWKinematicsSolvers() const
{
  return kinematics_information_.group_opw_kinematics;
}

bool ManipulatorManager::addGroupJointState(const std::string& group_name,
                                            const std::string& state_name,
                                            const tesseract_srdf::GroupsJointState& joint_state)
{
  if (!hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Tried to add Group State for group that does not exist!");
    return false;
  }

  kinematics_information_.group_states[group_name][state_name] = joint_state;
  // @TODO should check if within limits
  return true;
}

void ManipulatorManager::removeGroupJointState(const std::string& group_name, const std::string& state_name)
{
  kinematics_information_.group_states[group_name].erase(state_name);

  if (kinematics_information_.group_states[group_name].empty())
    kinematics_information_.group_states.erase(group_name);
}

bool ManipulatorManager::hasGroupJointState(const std::string& group_name, const std::string& state_name) const
{
  auto it = kinematics_information_.group_states.find(group_name);
  if (it == kinematics_information_.group_states.end())
    return false;

  return (it->second.find(state_name) != it->second.end());
}

const tesseract_srdf::GroupsJointState& ManipulatorManager::getGroupsJointState(const std::string& group_name,
                                                                                const std::string& state_name) const
{
  return kinematics_information_.group_states.at(group_name).at(state_name);
}

const tesseract_srdf::GroupsJointStates& ManipulatorManager::getGroupsJointStates(const std::string& group_name) const
{
  return kinematics_information_.group_states.at(group_name);
}

const tesseract_srdf::GroupJointStates& ManipulatorManager::getGroupJointStates() const
{
  return kinematics_information_.group_states;
}

bool ManipulatorManager::addGroupTCP(const std::string& group_name,
                                     const std::string& tcp_name,
                                     const Eigen::Isometry3d& tcp)
{
  if (!hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Tried to add Group TCP for group that does not exist!");
    return false;
  }

  kinematics_information_.group_tcps[group_name][tcp_name] = tcp;
  return true;
}

void ManipulatorManager::removeGroupTCP(const std::string& group_name, const std::string& tcp_name)
{
  kinematics_information_.group_tcps.at(group_name).erase(tcp_name);

  if (kinematics_information_.group_tcps[group_name].empty())
    kinematics_information_.group_tcps.erase(group_name);
}

bool ManipulatorManager::hasGroupTCP(const std::string& group_name, const std::string& tcp_name) const
{
  auto it = kinematics_information_.group_tcps.find(group_name);
  if (it == kinematics_information_.group_tcps.end())
    return false;

  return (it->second.find(tcp_name) != it->second.end());
}

const Eigen::Isometry3d& ManipulatorManager::getGroupsTCP(const std::string& group_name,
                                                          const std::string& tcp_name) const
{
  return kinematics_information_.group_tcps.at(group_name).at(tcp_name);
}

const tesseract_srdf::GroupsTCPs& ManipulatorManager::getGroupsTCPs(const std::string& group_name) const
{
  return kinematics_information_.group_tcps.at(group_name);
}

const tesseract_srdf::GroupTCPs& ManipulatorManager::getGroupTCPs() const { return kinematics_information_.group_tcps; }

bool ManipulatorManager::registerFwdKinematicsFactory(tesseract_kinematics::ForwardKinematicsFactory::ConstPtr factory)
{
  std::string name = factory->getName();
  if (fwd_kin_factories_.find(name) == fwd_kin_factories_.end())
  {
    fwd_kin_factories_[name] = std::move(factory);
    return true;
  }
  return false;
}

void ManipulatorManager::removeFwdKinematicsFactory(const std::string& name) { fwd_kin_factories_.erase(name); }

std::vector<std::string> ManipulatorManager::getAvailableFwdKinematicsSolvers() const
{
  std::vector<std::string> names;
  names.reserve(fwd_kin_factories_.size());
  for (const auto& factory : fwd_kin_factories_)
    names.push_back(factory.first);

  return names;
}

std::vector<std::string>
ManipulatorManager::getAvailableFwdKinematicsSolvers(tesseract_kinematics::ForwardKinematicsFactoryType type) const
{
  std::vector<std::string> names;
  names.reserve(fwd_kin_factories_.size());
  for (const auto& factory : fwd_kin_factories_)
    if (factory.second->getType() == type)
      names.push_back(factory.first);

  return names;
}

tesseract_kinematics::ForwardKinematicsFactory::ConstPtr
ManipulatorManager::getFwdKinematicFactory(const std::string& name) const
{
  auto it = fwd_kin_factories_.find(name);
  if (it != fwd_kin_factories_.end())
    return it->second;

  return nullptr;
}

bool ManipulatorManager::addFwdKinematicSolver(const tesseract_kinematics::ForwardKinematics::Ptr& solver)
{
  auto it = fwd_kin_manipulators_.find(std::make_pair(solver->getName(), solver->getSolverName()));
  if (it != fwd_kin_manipulators_.end())
    return false;

  fwd_kin_manipulators_[std::make_pair(solver->getName(), solver->getSolverName())] = solver;

  // If default solver does not exist for this manipulator set this solver as the default.
  auto it2 = fwd_kin_manipulators_default_.find(solver->getName());
  if (it2 == fwd_kin_manipulators_default_.end())
  {
    fwd_kin_manipulators_default_[solver->getName()] = solver;
    kinematics_information_.group_default_fwd_kin[solver->getName()] = solver->getSolverName();
  }

  return true;
}

void ManipulatorManager::removeFwdKinematicSolver(const std::string& manipulator, const std::string& name)
{
  fwd_kin_manipulators_.erase(std::make_pair(manipulator, name));
}

void ManipulatorManager::removeFwdKinematicSolver(const std::string& manipulator)
{
  auto it = fwd_kin_manipulators_.begin();
  while (it != fwd_kin_manipulators_.end())
  {
    if (it->first.first == manipulator)
      it = fwd_kin_manipulators_.erase(it);
    else
      ++it;
  }

  fwd_kin_manipulators_default_.erase(manipulator);
  kinematics_information_.group_default_fwd_kin.erase(manipulator);
}

std::vector<std::string> ManipulatorManager::getAvailableFwdKinematicsManipulators() const
{
  std::vector<std::string> names;
  names.reserve(fwd_kin_manipulators_default_.size());
  for (const auto& manip : fwd_kin_manipulators_default_)
    names.push_back(manip.first);

  return names;
}

bool ManipulatorManager::setDefaultFwdKinematicSolver(const std::string& manipulator, const std::string& name)
{
  auto it = fwd_kin_manipulators_.find(std::make_pair(manipulator, name));
  if (it == fwd_kin_manipulators_.end())
    return false;

  fwd_kin_manipulators_default_[manipulator] = it->second;
  kinematics_information_.group_default_fwd_kin[manipulator] = name;

  return true;
}

tesseract_kinematics::ForwardKinematics::Ptr ManipulatorManager::getFwdKinematicSolver(const std::string& manipulator,
                                                                                       const std::string& name) const
{
  auto it = fwd_kin_manipulators_.find(std::make_pair(manipulator, name));
  if (it != fwd_kin_manipulators_.end())
    return it->second->clone();

  return nullptr;
}

tesseract_kinematics::ForwardKinematics::Ptr
ManipulatorManager::getFwdKinematicSolver(const std::string& manipulator) const
{
  auto it = fwd_kin_manipulators_default_.find(manipulator);
  if (it != fwd_kin_manipulators_default_.end())
    return it->second->clone();

  return nullptr;
}

bool ManipulatorManager::registerInvKinematicsFactory(tesseract_kinematics::InverseKinematicsFactory::ConstPtr factory)
{
  std::string name = factory->getName();
  if (inv_kin_factories_.find(name) == inv_kin_factories_.end())
  {
    inv_kin_factories_[name] = std::move(factory);
    return true;
  }
  return false;
}

void ManipulatorManager::removeInvKinematicsFactory(const std::string& name) { inv_kin_factories_.erase(name); }

std::vector<std::string> ManipulatorManager::getAvailableInvKinematicsSolvers() const
{
  std::vector<std::string> names;
  names.reserve(inv_kin_factories_.size());
  for (const auto& factory : inv_kin_factories_)
    names.push_back(factory.first);

  return names;
}

std::vector<std::string>
ManipulatorManager::getAvailableInvKinematicsSolvers(tesseract_kinematics::InverseKinematicsFactoryType type) const
{
  std::vector<std::string> names;
  names.reserve(inv_kin_factories_.size());
  for (const auto& factory : inv_kin_factories_)
    if (factory.second->getType() == type)
      names.push_back(factory.first);

  return names;
}

tesseract_kinematics::InverseKinematicsFactory::ConstPtr
ManipulatorManager::getInvKinematicFactory(const std::string& name) const
{
  auto it = inv_kin_factories_.find(name);
  if (it != inv_kin_factories_.end())
    return it->second;

  return nullptr;
}

bool ManipulatorManager::addInvKinematicSolver(const tesseract_kinematics::InverseKinematics::Ptr& solver)
{
  if (solver->isSynchronized())
    throw std::runtime_error("ManipulatorManager does not accept inverse kinematics objects which have been "
                             "synchronized!");

  auto it = inv_kin_manipulators_.find(std::make_pair(solver->getName(), solver->getSolverName()));
  if (it != inv_kin_manipulators_.end())
    return false;

  inv_kin_manipulators_[std::make_pair(solver->getName(), solver->getSolverName())] = solver;

  // If default solver does not exist for this manipulator set this solver as the default.
  auto it2 = inv_kin_manipulators_default_.find(solver->getName());
  if (it2 == inv_kin_manipulators_default_.end())
  {
    inv_kin_manipulators_default_[solver->getName()] = solver;
    kinematics_information_.group_default_inv_kin[solver->getName()] = solver->getSolverName();
  }

  return true;
}

void ManipulatorManager::removeInvKinematicSolver(const std::string& manipulator, const std::string& name)
{
  inv_kin_manipulators_.erase(std::make_pair(manipulator, name));
}

void ManipulatorManager::removeInvKinematicSolver(const std::string& manipulator)
{
  auto it = inv_kin_manipulators_.begin();
  while (it != inv_kin_manipulators_.end())
  {
    if (it->first.first == manipulator)
      it = inv_kin_manipulators_.erase(it);
    else
      ++it;
  }

  inv_kin_manipulators_default_.erase(manipulator);
  kinematics_information_.group_default_inv_kin.erase(manipulator);
}

std::vector<std::string> ManipulatorManager::getAvailableInvKinematicsManipulators() const
{
  std::vector<std::string> names;
  names.reserve(inv_kin_manipulators_default_.size());
  for (const auto& manip : inv_kin_manipulators_default_)
    names.push_back(manip.first);

  return names;
}

bool ManipulatorManager::setDefaultInvKinematicSolver(const std::string& manipulator, const std::string& name)
{
  auto it = inv_kin_manipulators_.find(std::make_pair(manipulator, name));
  if (it == inv_kin_manipulators_.end())
    return false;

  inv_kin_manipulators_default_[manipulator] = it->second;
  kinematics_information_.group_default_inv_kin[manipulator] = name;

  return true;
}

tesseract_kinematics::InverseKinematics::Ptr ManipulatorManager::getInvKinematicSolver(const std::string& manipulator,
                                                                                       const std::string& name) const
{
  auto it = inv_kin_manipulators_.find(std::make_pair(manipulator, name));
  if (it != inv_kin_manipulators_.end())
    return it->second->clone();

  return nullptr;
}

tesseract_kinematics::InverseKinematics::Ptr
ManipulatorManager::getInvKinematicSolver(const std::string& manipulator) const
{
  auto it = inv_kin_manipulators_default_.find(manipulator);
  if (it != inv_kin_manipulators_default_.end())
    return it->second->clone();

  return nullptr;
}

bool ManipulatorManager::registerDefaultChainSolver(const std::string& group_name,
                                                    const tesseract_srdf::ChainGroup& chain_group)
{
  if (chain_group.empty())
    return false;

  auto fwd_solver = fwd_kin_chain_default_factory_->create(scene_graph_, chain_group, group_name);
  if (fwd_solver == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to create forward kinematic chain solver for manipulator %s!", group_name.c_str());
    return false;
  }

  if (!addFwdKinematicSolver(fwd_solver))
  {
    CONSOLE_BRIDGE_logError("Failed to add forward kinematic chain solver %s for manipulator %s to manager!",
                            fwd_solver->getSolverName().c_str(),
                            group_name.c_str());
    return false;
  }

  auto inv_solver = inv_kin_chain_default_factory_->create(scene_graph_, chain_group, group_name);
  if (inv_solver == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to create inverse kinematic chain solver for manipulator %s!", group_name.c_str());
    return false;
  }

  if (!addInvKinematicSolver(inv_solver))
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic chain solver %s for manipulator %s to manager!",
                            inv_solver->getSolverName().c_str(),
                            group_name.c_str());
    return false;
  }

  return true;
}

bool ManipulatorManager::registerDefaultJointSolver(const std::string& group_name,
                                                    const tesseract_srdf::JointGroup& joint_group)
{
  if (joint_group.empty())
    return false;

  auto solver = fwd_kin_tree_default_factory_->create(scene_graph_, joint_group, group_name);
  if (solver == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to create inverse kinematic tree solver %s for manipulator %s!",
                            solver->getSolverName().c_str(),
                            group_name.c_str());
    return false;
  }

  if (!addFwdKinematicSolver(solver))
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic tree solver %s for manipulator %s to manager!",
                            solver->getSolverName().c_str(),
                            group_name.c_str());
    return false;
  }

  CONSOLE_BRIDGE_logWarn("Joint groups are currently not supported by inverse kinematics!");
  return true;
}

bool ManipulatorManager::registerDefaultLinkSolver(const std::string& /*group_name*/,
                                                   const tesseract_srdf::LinkGroup& /*joint_group*/)
{
  CONSOLE_BRIDGE_logError("Link groups are currently not supported!");
  return false;
}

bool ManipulatorManager::registerOPWSolver(const std::string& group_name,
                                           const tesseract_srdf::OPWKinematicParameters& opw_params)
{
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin = getFwdKinematicSolver(group_name);
  if (fwd_kin == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic opw solver for manipulator %s to manager!",
                            group_name.c_str());
    return false;
  }

  opw_kinematics::Parameters<double> params;
  params.a1 = opw_params.a1;
  params.a2 = opw_params.a2;
  params.b = opw_params.b;
  params.c1 = opw_params.c1;
  params.c2 = opw_params.c2;
  params.c3 = opw_params.c3;
  params.c4 = opw_params.c4;
  for (std::size_t i = 0; i < 6; ++i)
  {
    params.offsets[i] = opw_params.offsets[i];
    params.sign_corrections[i] = opw_params.sign_corrections[i];
  }

  auto solver = std::make_shared<tesseract_kinematics::OPWInvKin>();
  solver->init(group_name,
               params,
               fwd_kin->getBaseLinkName(),
               fwd_kin->getTipLinkName(),
               fwd_kin->getJointNames(),
               fwd_kin->getLinkNames(),
               fwd_kin->getActiveLinkNames(),
               fwd_kin->getLimits());

  if (!solver->checkInitialized())
  {
    CONSOLE_BRIDGE_logError("Failed to create inverse kinematic opw solver for manipulator %s!", group_name.c_str());
    return false;
  }

  if (!addInvKinematicSolver(solver))
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic opw solver for manipulator %s to manager!",
                            group_name.c_str());
    return false;
  }

  // Automatically set OPW Inverse Kinematics as the default for the manipulator
  setDefaultInvKinematicSolver(solver->getName(), solver->getSolverName());

  return true;
}

bool ManipulatorManager::registerROPSolver(const std::string& group_name,
                                           const tesseract_srdf::ROPKinematicParameters& rop_group)
{
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin = getFwdKinematicSolver(group_name);
  if (fwd_kin == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic ROP solver for %s to manager!", group_name.c_str());
    return false;
  }

  tesseract_kinematics::InverseKinematics::Ptr manip_ik_solver =
      getInvKinematicSolver(rop_group.manipulator_group, rop_group.manipulator_ik_solver);

  if (manip_ik_solver == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to get manipulator inverse kinematics solver for ROP %s to manager!",
                            group_name.c_str());
    return false;
  }

  tesseract_kinematics::ForwardKinematics::Ptr positioner_fk_solver;
  if (rop_group.positioner_fk_solver.empty())
    positioner_fk_solver = getFwdKinematicSolver(rop_group.positioner_group);
  else
    positioner_fk_solver = getFwdKinematicSolver(rop_group.positioner_group, rop_group.positioner_fk_solver);

  if (positioner_fk_solver == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to get positioner forward kinematics solver for ROP %s to manager!",
                            group_name.c_str());
    return false;
  }

  // Extract Sampling resolutions
  const std::vector<std::string>& positioner_joints = positioner_fk_solver->getJointNames();
  Eigen::VectorXd positioner_sample_resolution(positioner_fk_solver->numJoints());
  for (std::size_t i = 0; i < positioner_joints.size(); ++i)
  {
    auto it = rop_group.positioner_sample_resolution.find(positioner_joints[i]);
    if (it == rop_group.positioner_sample_resolution.end())
    {
      CONSOLE_BRIDGE_logError("ManipulatorManager: Missing sampling resolution for joint: %s!",
                              positioner_joints[i].c_str());
      return false;
    }

    positioner_sample_resolution(static_cast<long>(i)) = it->second;
  }

  auto solver = std::make_shared<tesseract_kinematics::RobotOnPositionerInvKin>();
  if (rop_group.solver_name.empty())
  {
    if (!solver->init(scene_graph_,
                      manip_ik_solver,
                      rop_group.manipulator_reach,
                      positioner_fk_solver,
                      positioner_sample_resolution,
                      group_name))
      return false;
  }
  else
  {
    if (!solver->init(scene_graph_,
                      manip_ik_solver,
                      rop_group.manipulator_reach,
                      positioner_fk_solver,
                      positioner_sample_resolution,
                      group_name,
                      rop_group.solver_name))
      return false;
  }

  if (!addInvKinematicSolver(solver))
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic ROP solver for manipulator %s to manager!",
                            group_name.c_str());
    return false;
  }

  // Automatically set ROP Inverse Kinematics as the default for the manipulator
  setDefaultInvKinematicSolver(solver->getName(), solver->getSolverName());

  return true;
}

bool ManipulatorManager::registerREPSolver(const std::string& group_name,
                                           const tesseract_srdf::REPKinematicParameters& rep_group)
{
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin = getFwdKinematicSolver(group_name);
  if (fwd_kin == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic REP solver for %s to manager!", group_name.c_str());
    return false;
  }

  tesseract_kinematics::InverseKinematics::Ptr manip_ik_solver =
      getInvKinematicSolver(rep_group.manipulator_group, rep_group.manipulator_ik_solver);

  if (manip_ik_solver == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to get manipulator inverse kinematics solver for REP %s to manager!",
                            group_name.c_str());
    return false;
  }

  tesseract_kinematics::ForwardKinematics::Ptr positioner_fk_solver;
  if (rep_group.positioner_fk_solver.empty())
    positioner_fk_solver = getFwdKinematicSolver(rep_group.positioner_group);
  else
    positioner_fk_solver = getFwdKinematicSolver(rep_group.positioner_group, rep_group.positioner_fk_solver);

  if (positioner_fk_solver == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to get positioner forward kinematics solver for REP %s to manager!",
                            group_name.c_str());
    return false;
  }

  // Extract Sampling resolutions
  const std::vector<std::string>& positioner_joints = positioner_fk_solver->getJointNames();
  Eigen::VectorXd positioner_sample_resolution(positioner_fk_solver->numJoints());
  for (std::size_t i = 0; i < positioner_joints.size(); ++i)
  {
    auto it = rep_group.positioner_sample_resolution.find(positioner_joints[i]);
    if (it == rep_group.positioner_sample_resolution.end())
    {
      CONSOLE_BRIDGE_logError("ManipulatorManager: Missing sampling resolution for joint: %s!",
                              positioner_joints[i].c_str());
      return false;
    }

    positioner_sample_resolution(static_cast<long>(i)) = it->second;
  }

  auto solver = std::make_shared<tesseract_kinematics::RobotWithExternalPositionerInvKin>();
  if (rep_group.solver_name.empty())
  {
    if (!solver->init(scene_graph_,
                      manip_ik_solver,
                      rep_group.manipulator_reach,
                      positioner_fk_solver,
                      positioner_sample_resolution,
                      group_name))
      return false;
  }
  else
  {
    if (!solver->init(scene_graph_,
                      manip_ik_solver,
                      rep_group.manipulator_reach,
                      positioner_fk_solver,
                      positioner_sample_resolution,
                      group_name,
                      rep_group.solver_name))
      return false;
  }

  if (!addInvKinematicSolver(solver))
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic REP solver for manipulator %s to manager!",
                            group_name.c_str());
    return false;
  }

  // Automatically set ROP Inverse Kinematics as the default for the manipulator
  setDefaultInvKinematicSolver(solver->getName(), solver->getSolverName());

  return true;
}

}  // namespace tesseract_environment
