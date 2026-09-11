/**
 * @file trac-ik_factory_schema_unit.cpp
 * @brief Verify the schema the Trac-IK inverse kinematics factory registers
 *
 * @author Roelof Oomen
 * @date September 11, 2026
 *
 * @copyright Copyright (c) 2026, Roelof Oomen
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/property_tree.h>
#include <tesseract/common/schema_registry.h>
#include <tesseract/common/yaml_extensions.h>
#include <tesseract/kinematics/kinematics_plugin_factory.h>
#include <tesseract_trac_ik/trac-ik/trac-ik_factory.h>

using tesseract::common::KinematicsPluginInfo;
using tesseract::common::PropertyTree;

// The registration macros run at static initialization of the factory library; the anchor keeps
// that library linked so they run at all.
[[maybe_unused]] static const void* const trac_ik_anchor = tesseract::kinematics::TracIKFactoryAnchor();

namespace
{
/** @brief Every TRAC_IK::SolveType the factory accepts, in the order the schema declares them */
const std::vector<std::string> kSolveTypes{ "Speed", "Distance", "Manip1", "Manip2", "Manip3" };

std::string joinErrors(const std::vector<std::string>& errors)
{
  std::string msg;
  for (const auto& e : errors)
    msg += e + "\n";
  return msg;
}

/** @brief Validate a plugin config block against the registered schema */
std::vector<std::string> validate(const std::string& config)
{
  auto schema = YAML::convert<KinematicsPluginInfo>::schema();
  YAML::Node node = YAML::Load(R"(inv_kin_plugins:
  manipulator:
    default: TracIKInvKinChain
    plugins:
      TracIKInvKinChain:
        class: TracIKInvKinChainFactory
)");
  // Parse the config block as its own document so a caller writes plain YAML at column zero. Splicing
  // it in as text would make every caller responsible for one exact indentation, where a miscount
  // reparents the block under a neighbouring key instead of failing where the mistake is.
  node["inv_kin_plugins"]["manipulator"]["plugins"]["TracIKInvKinChain"]["config"] = YAML::Load(config);

  schema.mergeConfig(node);
  return schema.validate();
}
}  // namespace

TEST(TracIKFactorySchemaUnit, AcceptsMinimalConfig)  // NOLINT
{
  const auto errors = validate(R"(base_link: base_link
tip_link: tool0)");
  EXPECT_TRUE(errors.empty()) << joinErrors(errors);
}

TEST(TracIKFactorySchemaUnit, AcceptsEveryDocumentedKey)  // NOLINT
{
  const auto errors = validate(R"(base_link: base_link
tip_link: tool0
params:
  max_time: 0.5
  epsilon: 1e-4
  solve_type: Manip3
  bounds: [0.01, 0.02, 0.03, 0.1, 0.2, 0.3])");
  EXPECT_TRUE(errors.empty()) << joinErrors(errors);
}

TEST(TracIKFactorySchemaUnit, AcceptsEmptyParams)  // NOLINT
{
  // Every parameter is optional, so a params block that declares none of them is still a valid
  // config and create() falls back to the defaults throughout
  const auto errors = validate(R"(base_link: base_link
tip_link: tool0
params:)");
  EXPECT_TRUE(errors.empty()) << joinErrors(errors);
}

TEST(TracIKFactorySchemaUnit, RejectsMissingTipLink)  // NOLINT
{
  const auto errors = validate("base_link: base_link");
  ASSERT_FALSE(errors.empty());
  // Naming the key proves the rejection came from the required check, not from the factory being
  // unregistered, which rejects every config alike
  const std::string message = joinErrors(errors);
  EXPECT_NE(message.find("tip_link"), std::string::npos) << message;
  EXPECT_NE(message.find("required property missing or null"), std::string::npos) << message;
}

TEST(TracIKFactorySchemaUnit, RejectsEmptyBaseLink)  // NOLINT
{
  const auto errors = validate(R"(base_link: ""
tip_link: tool0)");
  ASSERT_FALSE(errors.empty());
  const std::string message = joinErrors(errors);
  EXPECT_NE(message.find("base_link"), std::string::npos) << message;
  EXPECT_NE(message.find("string length 0 is less than minimum 1"), std::string::npos) << message;
}

TEST(TracIKFactorySchemaUnit, RejectsEmptyTipLink)  // NOLINT
{
  const auto errors = validate(R"(base_link: base_link
tip_link: "")");
  ASSERT_FALSE(errors.empty());
  const std::string message = joinErrors(errors);
  EXPECT_NE(message.find("tip_link"), std::string::npos) << message;
  EXPECT_NE(message.find("string length 0 is less than minimum 1"), std::string::npos) << message;
}

TEST(TracIKFactorySchemaUnit, RejectsUnknownTopLevelKey)  // NOLINT
{
  const auto errors = validate(R"(base_link: base_link
tip_link: tool0
base_lnik: typo)");
  ASSERT_FALSE(errors.empty());
  const std::string message = joinErrors(errors);
  EXPECT_NE(message.find("base_lnik"), std::string::npos) << message;
  EXPECT_NE(message.find("property does not exist in schema"), std::string::npos) << message;
}

TEST(TracIKFactorySchemaUnit, RejectsUnknownKeyInParams)  // NOLINT
{
  const auto errors = validate(R"(base_link: base_link
tip_link: tool0
params:
  not_a_real_key: 1.0)");
  ASSERT_FALSE(errors.empty());
  const std::string message = joinErrors(errors);
  EXPECT_NE(message.find("not_a_real_key"), std::string::npos) << message;
  EXPECT_NE(message.find("property does not exist in schema"), std::string::npos) << message;
}

TEST(TracIKFactorySchemaUnit, RejectsUnknownSolveType)  // NOLINT
{
  const auto errors = validate(R"(base_link: base_link
tip_link: tool0
params:
  solve_type: Bogus)");
  ASSERT_FALSE(errors.empty());
  const std::string message = joinErrors(errors);
  EXPECT_NE(message.find("solve_type"), std::string::npos) << message;
  EXPECT_NE(message.find("not in enum list"), std::string::npos) << message;
}

TEST(TracIKFactorySchemaUnit, AcceptsEverySolveType)  // NOLINT
{
  for (const auto& solve_type : kSolveTypes)
  {
    const auto errors = validate(R"(base_link: base_link
tip_link: tool0
params:
  solve_type: )" + solve_type);
    EXPECT_TRUE(errors.empty()) << solve_type << ": " << joinErrors(errors);
  }
}

TEST(TracIKFactorySchemaUnit, RejectsNegativeMaxTime)  // NOLINT
{
  const auto errors = validate(R"(base_link: base_link
tip_link: tool0
params:
  max_time: -1.0)");
  ASSERT_FALSE(errors.empty());
  const std::string message = joinErrors(errors);
  EXPECT_NE(message.find("max_time"), std::string::npos) << message;
  EXPECT_NE(message.find("is less than minimum"), std::string::npos) << message;
}

TEST(TracIKFactorySchemaUnit, RejectsNegativeEpsilon)  // NOLINT
{
  const auto errors = validate(R"(base_link: base_link
tip_link: tool0
params:
  epsilon: -1e-5)");
  ASSERT_FALSE(errors.empty());
  const std::string message = joinErrors(errors);
  EXPECT_NE(message.find("epsilon"), std::string::npos) << message;
  EXPECT_NE(message.find("is less than minimum"), std::string::npos) << message;
}

TEST(TracIKFactorySchemaUnit, RejectsBoundsThatAreNotAFullTwist)  // NOLINT
{
  const auto errors = validate(R"(base_link: base_link
tip_link: tool0
params:
  bounds: [0.01, 0.02, 0.03])");
  ASSERT_FALSE(errors.empty());
  const std::string message = joinErrors(errors);
  EXPECT_NE(message.find("bounds"), std::string::npos) << message;
  EXPECT_NE(message.find("does not match expected 6"), std::string::npos) << message;
}

TEST(TracIKFactorySchemaUnit, AcceptsBoundsWhoseElementsAreNotNumbers)  // NOLINT
{
  // The sequence check covers length only, because float64 is not a registry key and so no element
  // validator is attached. Element types stay create()'s responsibility, which is why its own check
  // must not be removed in favour of the schema.
  const auto errors = validate(R"(base_link: base_link
tip_link: tool0
params:
  bounds: [a, b, c, d, e, f])");
  EXPECT_TRUE(errors.empty()) << joinErrors(errors);
}

TEST(TracIKFactorySchemaUnit, FactorySchemaMatchesTheRegisteredOne)  // NOLINT
{
  // Validation reads the registered schema and never this override, so the two can diverge without
  // any test of the loader noticing. Compare them including attributes, which is where the enum and
  // the numeric bounds live.
  const auto registry = tesseract::common::SchemaRegistry::instance();
  ASSERT_TRUE(registry->contains("TracIKInvKinChainFactory"));

  const tesseract::kinematics::TracIKInvKinChainFactory factory;
  const tesseract::kinematics::InvKinFactory& base = factory;

  EXPECT_EQ(YAML::Dump(base.schema().toYAML(false)),
            YAML::Dump(registry->get("TracIKInvKinChainFactory").toYAML(false)));
}

TEST(TracIKFactorySchemaUnit, SchemaDeclaresTheDocumentedKeys)  // NOLINT
{
  const tesseract::kinematics::TracIKInvKinChainFactory factory;
  const tesseract::kinematics::InvKinFactory& base = factory;
  const PropertyTree schema = base.schema();

  ASSERT_EQ(schema.size(), 3U);
  EXPECT_TRUE(schema.at("base_link").isRequired());
  EXPECT_TRUE(schema.at("tip_link").isRequired());

  const PropertyTree& params = schema.at("params");
  EXPECT_FALSE(params.isRequired());
  ASSERT_EQ(params.size(), 4U);
  const auto solve_type = params.at("solve_type").getAttribute(tesseract::common::property_attribute::ENUM);
  ASSERT_TRUE(solve_type.has_value());
  // NOLINTBEGIN(bugprone-unchecked-optional-access)
  ASSERT_EQ(solve_type->size(), 5U);
  EXPECT_EQ(solve_type->as<std::vector<std::string>>(), kSolveTypes);
  // NOLINTEND(bugprone-unchecked-optional-access)
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
