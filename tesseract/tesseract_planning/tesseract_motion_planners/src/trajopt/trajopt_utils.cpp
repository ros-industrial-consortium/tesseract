/**
 * @file trajopt_utils.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>

namespace tesseract_planning
{
trajopt::TermInfo::Ptr createCartesianWaypointTermInfo(const Eigen::Isometry3d& c_wp,
                                                       int index,
                                                       std::string working_frame,
                                                       Eigen::Isometry3d tcp,
                                                       const Eigen::VectorXd& coeffs,
                                                       std::string link,
                                                       trajopt::TermType type)
{
  auto pose_info = std::make_shared<trajopt::CartPoseTermInfo>();
  pose_info->term_type = type;
  pose_info->name = "cartesian_waypoint_" + std::to_string(index);

  pose_info->link = link;
  pose_info->tcp = tcp;

  pose_info->timestep = index;
  pose_info->xyz = c_wp.translation();
  Eigen::Quaterniond q(c_wp.linear());
  pose_info->wxyz = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
  pose_info->target = working_frame;

  if (coeffs.size() == 1)
  {
    pose_info->pos_coeffs = Eigen::Vector3d::Constant(coeffs(0));
    pose_info->rot_coeffs = Eigen::Vector3d::Constant(coeffs(0));
  }
  else if (coeffs.size() == 6)
  {
    pose_info->pos_coeffs = coeffs.head<3>();
    pose_info->rot_coeffs = coeffs.tail<3>();
  }

  return pose_info;
}

trajopt::TermInfo::Ptr createDynamicCartesianWaypointTermInfo(const Eigen::Isometry3d& c_wp,
                                                              int index,
                                                              std::string working_frame,
                                                              const Eigen::Isometry3d& tcp,
                                                              const Eigen::VectorXd& coeffs,
                                                              const std::string& link,
                                                              trajopt::TermType type)
{
  std::shared_ptr<trajopt::DynamicCartPoseTermInfo> pose = std::make_shared<trajopt::DynamicCartPoseTermInfo>();
  pose->term_type = type;
  pose->name = "dyn_cartesian_waypoint_" + std::to_string(index);
  pose->timestep = index;

  pose->link = link;
  pose->tcp = tcp;

  pose->target = working_frame;
  pose->target_tcp = c_wp;

  if (coeffs.size() == 1)
  {
    pose->pos_coeffs = Eigen::Vector3d::Constant(coeffs(0));
    pose->rot_coeffs = Eigen::Vector3d::Constant(coeffs(0));
  }
  else if (coeffs.size() == 6)
  {
    pose->pos_coeffs = coeffs.head<3>();
    pose->rot_coeffs = coeffs.tail<3>();
  }

  return pose;
}

trajopt::TermInfo::Ptr createNearJointStateTermInfo(const Eigen::VectorXd& target,
                                                    const std::vector<std::string>& joint_names,
                                                    int index,
                                                    const Eigen::VectorXd& coeffs,
                                                    trajopt::TermType type)
{
  assert(static_cast<std::size_t>(target.size()) == joint_names.size());

  std::shared_ptr<trajopt::JointPosTermInfo> jp = std::make_shared<trajopt::JointPosTermInfo>();
  if (static_cast<std::size_t>(coeffs.size()) == 1)
    jp->coeffs = std::vector<double>(joint_names.size(), coeffs(0));  // Default value
  else if (static_cast<std::size_t>(coeffs.size()) == joint_names.size())
    jp->coeffs = std::vector<double>(coeffs.data(), coeffs.data() + coeffs.rows() * coeffs.cols());

  jp->targets = std::vector<double>(target.data(), target.data() + target.size());
  jp->first_step = index;
  jp->last_step = index;
  jp->name = "near_state_" + std::to_string(index);
  jp->term_type = type;

  return jp;
}

trajopt::TermInfo::Ptr createJointWaypointTermInfo(const Eigen::VectorXd& j_wp,
                                                   int index,
                                                   const Eigen::VectorXd& coeffs,
                                                   trajopt::TermType type)
{
  auto joint_info = std::make_shared<trajopt::JointPosTermInfo>();
  if (coeffs.size() == 1)
    joint_info->coeffs = std::vector<double>(static_cast<std::size_t>(j_wp.size()), coeffs(0));
  else if (coeffs.size() == j_wp.size())
    joint_info->coeffs = std::vector<double>(coeffs.data(), coeffs.data() + coeffs.rows() * coeffs.cols());

  joint_info->targets = std::vector<double>(j_wp.data(), j_wp.data() + j_wp.rows() * j_wp.cols());
  joint_info->first_step = index;
  joint_info->last_step = index;
  joint_info->name = "joint_waypoint_" + std::to_string(index);
  joint_info->term_type = type;

  return joint_info;
}

trajopt::TermInfo::Ptr createTolerancedJointWaypointTermInfo(const Eigen::VectorXd& j_wp,
                                                             const Eigen::VectorXd& lower_tol,
                                                             const Eigen::VectorXd& upper_tol,
                                                             int index,
                                                             const Eigen::VectorXd& coeffs,
                                                             trajopt::TermType type)
{
  auto joint_info = std::make_shared<trajopt::JointPosTermInfo>();
  if (coeffs.size() == 1)
    joint_info->coeffs = std::vector<double>(static_cast<std::size_t>(j_wp.size()), coeffs(0));
  else if (coeffs.size() == j_wp.size())
    joint_info->coeffs = std::vector<double>(coeffs.data(), coeffs.data() + coeffs.rows() * coeffs.cols());

  joint_info->targets = std::vector<double>(j_wp.data(), j_wp.data() + j_wp.rows() * j_wp.cols());
  joint_info->lower_tols =
      std::vector<double>(lower_tol.data(), lower_tol.data() + lower_tol.rows() * lower_tol.cols());
  joint_info->upper_tols =
      std::vector<double>(upper_tol.data(), upper_tol.data() + upper_tol.rows() * upper_tol.cols());
  joint_info->first_step = index;
  joint_info->last_step = index;
  joint_info->name = "joint_waypoint_" + std::to_string(index);
  joint_info->term_type = type;

  return joint_info;
}

trajopt::TermInfo::Ptr createCollisionTermInfo(int start_index,
                                               int end_index,
                                               double collision_safety_margin,
                                               double collision_safety_margin_buffer,
                                               trajopt::CollisionEvaluatorType evaluator_type,
                                               bool use_weighted_sum,
                                               double coeff,
                                               tesseract_collision::ContactTestType contact_test_type,
                                               double longest_valid_segment_length,
                                               trajopt::TermType type)
{
  std::shared_ptr<trajopt::CollisionTermInfo> collision = std::make_shared<trajopt::CollisionTermInfo>();
  collision->name = "collision";
  collision->term_type = type;
  collision->evaluator_type = evaluator_type;
  collision->use_weighted_sum = use_weighted_sum;
  collision->first_step = start_index;
  collision->last_step = end_index;
  collision->contact_test_type = contact_test_type;
  collision->longest_valid_segment_length = longest_valid_segment_length;
  collision->info =
      trajopt::createSafetyMarginDataVector((end_index - start_index) + 1, collision_safety_margin, coeff);
  collision->safety_margin_buffer = collision_safety_margin_buffer;
  return collision;
}

trajopt::TermInfo::Ptr
createSmoothVelocityTermInfo(int start_index, int end_index, int n_joints, double coeff, trajopt::TermType type)
{
  if ((end_index - start_index) < 2)
    throw std::runtime_error("TrajOpt JointVelTermInfo requires at least two states!");

  std::shared_ptr<trajopt::JointVelTermInfo> jv = std::make_shared<trajopt::JointVelTermInfo>();
  jv->coeffs = std::vector<double>(static_cast<std::size_t>(n_joints), coeff);
  jv->targets = std::vector<double>(static_cast<std::size_t>(n_joints), 0.0);
  jv->first_step = start_index;
  jv->last_step = end_index;
  jv->name = "joint_vel_cost";
  jv->term_type = type;
  return jv;
}

trajopt::TermInfo::Ptr createSmoothVelocityTermInfo(int start_index,
                                                    int end_index,
                                                    const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                    trajopt::TermType type)
{
  if ((end_index - start_index) < 2)
    throw std::runtime_error("TrajOpt JointVelTermInfo requires at least two states!");

  std::shared_ptr<trajopt::JointVelTermInfo> jv = std::make_shared<trajopt::JointVelTermInfo>();
  jv->coeffs = std::vector<double>(coeff.data(), coeff.data() + coeff.size());
  jv->targets = std::vector<double>(static_cast<std::size_t>(coeff.size()), 0.0);
  jv->first_step = start_index;
  jv->last_step = end_index;
  jv->name = "joint_vel_cost";
  jv->term_type = type;
  return jv;
}

trajopt::TermInfo::Ptr
createSmoothAccelerationTermInfo(int start_index, int end_index, int n_joints, double coeff, trajopt::TermType type)
{
  if ((end_index - start_index) < 3)
    throw std::runtime_error("TrajOpt JointAccTermInfo requires at least three states!");

  std::shared_ptr<trajopt::JointAccTermInfo> ja = std::make_shared<trajopt::JointAccTermInfo>();
  ja->coeffs = std::vector<double>(static_cast<std::size_t>(n_joints), coeff);
  ja->targets = std::vector<double>(static_cast<std::size_t>(n_joints), 0.0);
  ja->first_step = start_index;
  ja->last_step = end_index;
  ja->name = "joint_accel_cost";
  ja->term_type = type;
  return ja;
}

trajopt::TermInfo::Ptr createSmoothAccelerationTermInfo(int start_index,
                                                        int end_index,
                                                        const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                        trajopt::TermType type)
{
  if ((end_index - start_index) < 3)
    throw std::runtime_error("TrajOpt JointAccTermInfo requires at least three states!");

  std::shared_ptr<trajopt::JointAccTermInfo> ja = std::make_shared<trajopt::JointAccTermInfo>();
  ja->coeffs = std::vector<double>(coeff.data(), coeff.data() + coeff.size());
  ja->targets = std::vector<double>(static_cast<std::size_t>(coeff.size()), 0.0);
  ja->first_step = start_index;
  ja->last_step = end_index;
  ja->name = "joint_accel_cost";
  ja->term_type = type;
  return ja;
}

trajopt::TermInfo::Ptr
createSmoothJerkTermInfo(int start_index, int end_index, int n_joints, double coeff, trajopt::TermType type)
{
  if ((end_index - start_index) < 5)
    throw std::runtime_error("TrajOpt JointJerkTermInfo requires at least five states!");

  std::shared_ptr<trajopt::JointJerkTermInfo> jj = std::make_shared<trajopt::JointJerkTermInfo>();
  jj->coeffs = std::vector<double>(static_cast<std::size_t>(n_joints), coeff);
  jj->targets = std::vector<double>(static_cast<std::size_t>(n_joints), 0.0);
  jj->first_step = start_index;
  jj->last_step = end_index;
  jj->name = "joint_jerk_cost";
  jj->term_type = type;
  return jj;
}

trajopt::TermInfo::Ptr createSmoothJerkTermInfo(int start_index,
                                                int end_index,
                                                const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                trajopt::TermType type)
{
  if ((end_index - start_index) < 5)
    throw std::runtime_error("TrajOpt JointJerkTermInfo requires at least five states!");

  std::shared_ptr<trajopt::JointJerkTermInfo> jj = std::make_shared<trajopt::JointJerkTermInfo>();
  jj->coeffs = std::vector<double>(coeff.data(), coeff.data() + coeff.size());
  jj->targets = std::vector<double>(static_cast<std::size_t>(coeff.size()), 0.0);
  jj->first_step = start_index;
  jj->last_step = end_index;
  jj->name = "joint_jerk_cost";
  jj->term_type = type;
  return jj;
}

trajopt::TermInfo::Ptr createUserDefinedTermInfo(int start_index,
                                                 int end_index,
                                                 sco::VectorOfVector::func error_function,
                                                 sco::MatrixOfVector::func jacobian_function,
                                                 trajopt::TermType type)
{
  if (error_function == nullptr)
  {
    throw std::runtime_error("TrajOpt Planner Config constraint from error function recieved nullptr!");
  }

  auto ef = std::make_shared<trajopt::UserDefinedTermInfo>();
  ef->name = "user_defined";
  ef->term_type = type;
  ef->first_step = start_index;
  ef->last_step = end_index;
  ef->error_function = std::move(error_function);
  ef->jacobian_function = std::move(jacobian_function);

  return ef;
}

trajopt::TermInfo::Ptr createAvoidSingularityTermInfo(int start_index,
                                                      int end_index,
                                                      const std::string& link,
                                                      double coeff,
                                                      trajopt::TermType type)
{
  auto as = std::make_shared<trajopt::AvoidSingularityTermInfo>();
  as->term_type = type;
  as->link = link;
  as->first_step = start_index;
  as->last_step = end_index;
  as->coeffs = std::vector<double>(1, coeff);
  as->name = "avoid_singularity";

  return as;
}

}  // namespace tesseract_planning
