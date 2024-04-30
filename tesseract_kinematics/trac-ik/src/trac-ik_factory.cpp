/**
 * @file trac-ik_factory.h
 * @brief Tesseract Trac-IK Factory.
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

#include <tesseract_trac_ik/trac-ik/trac-ik_factory.h>
#include <tesseract_trac_ik/trac-ik/trac-ik_inv_kin_chain.h>

namespace tesseract_kinematics
{
std::unique_ptr<InverseKinematics>
TracIKInvKinChainFactory::create(const std::string& solver_name,
                                 const tesseract_scene_graph::SceneGraph& scene_graph,
                                 const tesseract_scene_graph::SceneState& /*scene_state*/,
                                 const KinematicsPluginFactory& /*plugin_factory*/,
                                 const YAML::Node& config) const
{
  std::string base_link;
  std::string tip_link;
  double max_time = MAX_TIME;
  double epsilon = EPSILON;
  TRAC_IK::SolveType solve_type = SOLVE_TYPE;

  try
  {
    if (YAML::Node n = config["base_link"])
      base_link = n.as<std::string>();
    else
      throw std::runtime_error("TracIKInvKinChainFactory, missing 'base_link' entry");

    if (YAML::Node n = config["tip_link"])
      tip_link = n.as<std::string>();
    else
      throw std::runtime_error("TracIKInvKinChainFactory, missing 'tip_link' entry");

    if (YAML::Node opw_params = config["params"])
    {
      if (YAML::Node n = opw_params["max_time"])
      {
        max_time = n.as<double>();
      }
      if (YAML::Node n = opw_params["epsilon"])
      {
        epsilon = n.as<double>();
      }
      if (YAML::Node n = opw_params["solve_type"])
      {
        if (n.as<std::string>() == "Speed")
        {
          solve_type = TRAC_IK::SolveType::Speed;
        }
        else if (n.as<std::string>() == "Distance")
        {
          solve_type = TRAC_IK::SolveType::Distance;
        }
        else if (n.as<std::string>() == "Manip1")
        {
          solve_type = TRAC_IK::SolveType::Manip1;
        }
        else if (n.as<std::string>() == "Manip2")
        {
          solve_type = TRAC_IK::SolveType::Manip2;
        }
        else
        {
          throw std::runtime_error("TracIKInvKinChainFactory, 'params' entry 'solve_type' invalid");
        }
      }
    }
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("TracIKInvKinChainFactory: Failed to parse yaml config data! Details: %s", e.what());
    return nullptr;
  }

  return std::make_unique<TracIKInvKinChain>(
      scene_graph, base_link, tip_link, solver_name, max_time, epsilon, solve_type);
}

TESSERACT_PLUGIN_ANCHOR_IMPL(TracIKFactoryAnchor)

}  // namespace tesseract_kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract_kinematics::TracIKInvKinChainFactory, TracIKInvKinChainFactory);
