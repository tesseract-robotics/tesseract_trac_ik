/**
 * @file trac-ik_fwd_kin_chain.cpp
 * @brief Tesseract Trac-IK Inverse kinematics chain implementation.
 *
 * @author Roelof Oomen
 * @date July 25, 2023
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Southwest Research Institute
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
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_trac_ik/trac-ik/trac-ik_inv_kin_chain.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

namespace tesseract_kinematics
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

TracIKInvKinChain::TracIKInvKinChain(const tesseract_scene_graph::SceneGraph& scene_graph,
                                     const std::vector<std::pair<std::string, std::string>>& chains,
                                     std::string solver_name,
                                     double max_time,
                                     double epsilon,
                                     TRAC_IK::SolveType solve_type)
  : max_time_(max_time), epsilon_(epsilon), solve_type_(solve_type), solver_name_(std::move(solver_name))
{
  if (!scene_graph.getLink(scene_graph.getRoot()))
    throw std::runtime_error("The scene graph has an invalid root");

  if (!parseSceneGraph(kdl_data_, scene_graph, chains))
    throw std::runtime_error("Failed to parse KDL data from scene graph");

  // Create Trac-IK IK Solver
  ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
      kdl_data_.robot_chain, kdl_data_.q_min, kdl_data_.q_max, max_time_, epsilon_, solve_type_);
}

TracIKInvKinChain::TracIKInvKinChain(const tesseract_scene_graph::SceneGraph& scene_graph,
                                     const std::string& base_link,
                                     const std::string& tip_link,
                                     std::string solver_name,
                                     double max_time,
                                     double epsilon,
                                     TRAC_IK::SolveType solve_type)
  : TracIKInvKinChain(scene_graph,
                      { std::make_pair(base_link, tip_link) },
                      std::move(solver_name),
                      max_time,
                      epsilon,
                      solve_type)
{
}

InverseKinematics::UPtr TracIKInvKinChain::clone() const { return std::make_unique<TracIKInvKinChain>(*this); }

TracIKInvKinChain::TracIKInvKinChain(const TracIKInvKinChain& other) { *this = other; }

TracIKInvKinChain& TracIKInvKinChain::operator=(const TracIKInvKinChain& other)
{
  kdl_data_ = other.kdl_data_;
  max_time_ = other.max_time_;
  epsilon_ = other.epsilon_;
  solve_type_ = other.solve_type_;
  ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
      kdl_data_.robot_chain, kdl_data_.q_min, kdl_data_.q_max, max_time_, epsilon_, solve_type_);
  solver_name_ = other.solver_name_;

  return *this;
}

IKSolutions TracIKInvKinChain::calcInvKinHelper(const Eigen::Isometry3d& pose,
                                                const Eigen::Ref<const Eigen::VectorXd>& seed,
                                                int /*segment_num*/) const
{
  assert(std::abs(1.0 - pose.matrix().determinant()) < 1e-6);  // NOLINT
  KDL::JntArray kdl_seed, kdl_solution;
  EigenToKDL(seed, kdl_seed);
  kdl_solution.resize(static_cast<unsigned>(seed.size()));
  Eigen::VectorXd solution(seed.size());

  // Run IK solver
  KDL::Frame kdl_pose;
  EigenToKDL(pose, kdl_pose);
  int status{ -1 };
  {
    std::lock_guard<std::mutex> guard(mutex_);
    status = ik_solver_->CartToJnt(kdl_seed, kdl_pose, kdl_solution);
  }
  if (status < 0)
  {
    if (status == -3)
    {
      CONSOLE_BRIDGE_logDebug("Trac-IK did not find any solutions");
    }
    else
    {
      CONSOLE_BRIDGE_logDebug("Trac-IK failed to calculate IK");
    }
    return {};
  }

  KDLToEigen(kdl_solution, solution);

  return { solution };
}

IKSolutions TracIKInvKinChain::calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                                          const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  assert(tip_link_poses.find(kdl_data_.tip_link_name) != tip_link_poses.end());
  return calcInvKinHelper(tip_link_poses.at(kdl_data_.tip_link_name), seed);
}

std::vector<std::string> TracIKInvKinChain::getJointNames() const { return kdl_data_.joint_names; }

Eigen::Index TracIKInvKinChain::numJoints() const { return kdl_data_.robot_chain.getNrOfJoints(); }

std::string TracIKInvKinChain::getBaseLinkName() const { return kdl_data_.base_link_name; }

std::string TracIKInvKinChain::getWorkingFrame() const { return kdl_data_.base_link_name; }

std::vector<std::string> TracIKInvKinChain::getTipLinkNames() const { return { kdl_data_.tip_link_name }; }

std::string TracIKInvKinChain::getSolverName() const { return solver_name_; }

}  // namespace tesseract_kinematics
