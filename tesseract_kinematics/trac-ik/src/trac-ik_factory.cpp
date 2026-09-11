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

#include <tesseract/common/property_tree.h>
#include <tesseract/common/schema_registration.h>

#include <array>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
// Every TRAC_IK::SolveType the factory accepts. The schema's enum and create()'s lookup both read
// this table, so a solver type cannot be declared in one and forgotten in the other.
constexpr std::array<std::pair<std::string_view, TRAC_IK::SolveType>, 5> SOLVE_TYPES{
  { { "Speed", TRAC_IK::SolveType::Speed },
    { "Distance", TRAC_IK::SolveType::Distance },
    { "Manip1", TRAC_IK::SolveType::Manip1 },
    { "Manip2", TRAC_IK::SolveType::Manip2 },
    { "Manip3", TRAC_IK::SolveType::Manip3 } }
};

/** @brief Return the solve type registered under this name, or nullptr if there is none */
const TRAC_IK::SolveType* solveTypeNamed(std::string_view name)
{
  for (const auto& entry : SOLVE_TYPES)
  {
    if (entry.first == name)
      return &entry.second;
  }
  return nullptr;
}

std::vector<std::string> solveTypeNames()
{
  std::vector<std::string> names;
  names.reserve(SOLVE_TYPES.size());
  for (const auto& [name, type] : SOLVE_TYPES)
    names.emplace_back(name);
  return names;
}

tesseract::common::PropertyTree tracIKInvKinChainFactorySchema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .string("base_link").required().minimumLength(1).done()
      .string("tip_link").required().minimumLength(1).done()
      .container("params")
          .float64("max_time").minimum(0.0).done()
          .float64("epsilon").minimum(0.0).done()
          .string("solve_type").enumValues(solveTypeNames()).done()
          // A full twist: linear x/y/z then angular x/y/z
          .customType("bounds", property_type::createList(property_type::FLOAT64, 6)).done()
          .done()
      .build();
  // clang-format on
}
}  // namespace

namespace tesseract::kinematics
{
tesseract::common::PropertyTree TracIKInvKinChainFactory::schema() const { return tracIKInvKinChainFactorySchema(); }

std::unique_ptr<InverseKinematics>
TracIKInvKinChainFactory::create(const std::string& solver_name,
                                 const tesseract::scene_graph::SceneGraph& scene_graph,
                                 const tesseract::scene_graph::SceneState& /*scene_state*/,
                                 const KinematicsPluginFactory& /*plugin_factory*/,
                                 const YAML::Node& config) const
{
  common::LinkId base_link;
  common::LinkId tip_link;
  double max_time = MAX_TIME;
  double epsilon = EPSILON;
  TRAC_IK::SolveType solve_type = SOLVE_TYPE;
  KDL::Twist bounds = BOUNDS;

  try
  {
    if (const YAML::Node& n = config["base_link"])
      base_link = common::LinkId(n.as<std::string>());
    else
      throw std::runtime_error("TracIKInvKinChainFactory, missing 'base_link' entry");

    if (const YAML::Node& n = config["tip_link"])
      tip_link = common::LinkId(n.as<std::string>());
    else
      throw std::runtime_error("TracIKInvKinChainFactory, missing 'tip_link' entry");

    if (const YAML::Node& params = config["params"])
    {
      if (const YAML::Node& n = params["max_time"])
      {
        max_time = n.as<double>();
      }
      if (const YAML::Node& n = params["epsilon"])
      {
        epsilon = n.as<double>();
      }
      if (const YAML::Node& n = params["solve_type"])
      {
        const auto type = n.as<std::string>();
        const auto* match = solveTypeNamed(type);
        if (match == nullptr)
          throw std::runtime_error("TracIKInvKinChainFactory, 'params' entry 'solve_type' invalid");
        solve_type = *match;
      }
      if (const YAML::Node& n = params["bounds"])
      {
        const auto v = n.as<std::vector<double>>();
        if (v.size() != 6)
          throw std::runtime_error("TracIKInvKinChainFactory, 'params' entry 'bounds' must have 6 elements");
        bounds = KDL::Twist(KDL::Vector(v[0], v[1], v[2]), KDL::Vector(v[3], v[4], v[5]));
      }
    }

    return std::make_unique<TracIKInvKinChain>(
        scene_graph, base_link, tip_link, solver_name, max_time, epsilon, solve_type, bounds);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("TracIKInvKinChainFactory: Failed to parse yaml config data! Details: %s", e.what());
    return nullptr;
  }
}

PLUGIN_ANCHOR_IMPL(TracIKFactoryAnchor)

}  // namespace tesseract::kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract::kinematics::TracIKInvKinChainFactory, TracIKInvKinChainFactory);
TESSERACT_SCHEMA_REGISTER(TracIKInvKinChainFactory, tracIKInvKinChainFactorySchema);
TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::kinematics::InvKinFactory, TracIKInvKinChainFactory);
