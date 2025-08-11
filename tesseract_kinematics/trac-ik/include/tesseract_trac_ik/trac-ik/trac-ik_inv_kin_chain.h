/**
 * @file trac-ik_inv_kin_chain.h
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
#ifndef TESSERACT_KINEMATICS_TRACIK_INV_KIN_CHAIN_H
#define TESSERACT_KINEMATICS_TRACIK_INV_KIN_CHAIN_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <mutex>
#include <trac_ik/trac_ik.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/types.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

namespace tesseract_kinematics
{
static const std::string TRACIK_INV_KIN_CHAIN_SOLVER_NAME = "TracIKInvKinChain";
static const double MAX_TIME = 0.005;
static const double EPSILON = 1e-5;
static const TRAC_IK::SolveType SOLVE_TYPE = TRAC_IK::SolveType::Speed;

/**
 * @brief KDL Inverse kinematic chain implementation.
 */
class TracIKInvKinChain : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<TracIKInvKinChain>;
  using ConstPtr = std::shared_ptr<const TracIKInvKinChain>;
  using UPtr = std::unique_ptr<TracIKInvKinChain>;
  using ConstUPtr = std::unique_ptr<const TracIKInvKinChain>;

  ~TracIKInvKinChain() override = default;
  TracIKInvKinChain(const TracIKInvKinChain& other);
  TracIKInvKinChain& operator=(const TracIKInvKinChain& other);
  TracIKInvKinChain(TracIKInvKinChain&&) = delete;
  TracIKInvKinChain& operator=(TracIKInvKinChain&&) = delete;

  /**
   * @brief Construct Inverse Kinematics as chain
   * Creates a inverse kinematic chain object
   * @param scene_graph The Tesseract Scene Graph
   * @param base_link The name of the base link for the kinematic chain
   * @param tip_link The name of the tip link for the kinematic chain
   * @param solver_name The name of the kinematic chain
   */
  TracIKInvKinChain(const tesseract_scene_graph::SceneGraph& scene_graph,
                    const std::string& base_link,
                    const std::string& tip_link,
                    std::string solver_name = TRACIK_INV_KIN_CHAIN_SOLVER_NAME,
                    double max_time = MAX_TIME,
                    double epsilon = EPSILON,
                    TRAC_IK::SolveType solve_type = SOLVE_TYPE);

  /**
   * @brief Construct Inverse Kinematics as chain
   * Creates a inverse kinematic chain object from sequential chains
   * @param scene_graph The Tesseract Scene Graph
   * @param chains A vector of kinematics chains <base_link, tip_link> that get concatenated
   * @param solver_name The solver name of the kinematic chain
   */
  TracIKInvKinChain(const tesseract_scene_graph::SceneGraph& scene_graph,
                    const std::vector<std::pair<std::string, std::string> >& chains,
                    std::string solver_name = TRACIK_INV_KIN_CHAIN_SOLVER_NAME,
                    double max_time = MAX_TIME,
                    double epsilon = EPSILON,
                    TRAC_IK::SolveType solve_type = SOLVE_TYPE);
  void calcInvKin(IKSolutions& solutions,
                  const tesseract_common::TransformMap& tip_link_poses,
                  const Eigen::Ref<const Eigen::VectorXd>& seed) const override final;

  std::vector<std::string> getJointNames() const override final;
  Eigen::Index numJoints() const override final;
  std::string getBaseLinkName() const override final;
  std::string getWorkingFrame() const override final;
  std::vector<std::string> getTipLinkNames() const override final;
  std::string getSolverName() const override final;
  InverseKinematics::UPtr clone() const override final;

private:
  double max_time_{ MAX_TIME };
  double epsilon_{ EPSILON };
  TRAC_IK::SolveType solve_type_{ SOLVE_TYPE };
  KDLChainData kdl_data_;                                       /**< @brief KDL data parsed from Scene Graph */
  std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_;                 /**< @brief Trac-IK Inverse kinematic solver */
  std::string solver_name_{ TRACIK_INV_KIN_CHAIN_SOLVER_NAME }; /**< @brief Name of this solver */
  mutable std::mutex mutex_; /**< @brief KDL is not thread safe due to mutable variables in Joint Class */
  // A fix exists, but no new release of KDL has been made in the meantime (KDL is still at v1.5.1 from Sep/2021):
  // https://github.com/orocos/orocos_kinematics_dynamics/pull/399

  /** @brief calcFwdKin helper function */
  void calcInvKinHelper(IKSolutions& solutions,
                        const Eigen::Isometry3d& pose,
                        const Eigen::Ref<const Eigen::VectorXd>& seed,
                        int segment_num = -1) const;
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_TRACIK_INV_KIN_CHAIN_H
