#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_trac_ik/trac-ik/trac-ik_factory.h>
#include <tesseract_trac_ik/trac-ik/trac-ik_inv_kin_chain.h>

#include <tesseract/kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract/kinematics/kinematics_plugin_factory.h>
#include <tesseract/common/property_tree.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <tesseract/urdf/urdf_parser.h>

using namespace tesseract::kinematics;
using tesseract::common::LinkId;

namespace
{
const LinkId BASE_LINK{ "base_link" };
const LinkId TIP_LINK{ "tool0" };

struct Fixture
{
  Fixture()
  {
    tesseract::common::GeneralResourceLocator locator;
    const std::string path =
        locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath();
    scene_graph = tesseract::urdf::parseURDFFile(path, locator);

    const tesseract::scene_graph::KDLStateSolver state_solver(*scene_graph);
    scene_state = state_solver.getState();
  }

  tesseract::scene_graph::SceneGraph::UPtr scene_graph;
  tesseract::scene_graph::SceneState scene_state;
};

/** @brief Create through the factory directly, bypassing plugin loading */
std::unique_ptr<InverseKinematics> create(const Fixture& fixture, const std::string& config)
{
  const TracIKInvKinChainFactory factory;
  const KinematicsPluginFactory plugin_factory;
  return factory.create(
      TRACIK_INV_KIN_CHAIN_SOLVER_NAME, *fixture.scene_graph, fixture.scene_state, plugin_factory, YAML::Load(config));
}

/** @brief Expose createImpl, so a test can hand the factory a config that failed validation */
class UnvalidatedTracIKInvKinChainFactory : public TracIKInvKinChainFactory
{
public:
  using TracIKInvKinChainFactory::createImpl;
};

/** @brief Expect two twists to match component by component, linear x/y/z then angular x/y/z */
void expectTwistEq(const KDL::Twist& actual, const KDL::Twist& expected)
{
  for (int i = 0; i < 6; ++i)
    EXPECT_DOUBLE_EQ(actual(i), expected(i)) << "twist component " << i;
}
}  // namespace

TEST(TesseractKinematicsFactoryUnit, TracIKCreateUnit)  // NOLINT
{
  const Fixture fixture;
  auto inv_kin = create(fixture, R"(base_link: base_link
tip_link: tool0)");

  ASSERT_TRUE(inv_kin != nullptr);
  EXPECT_EQ(inv_kin->getSolverName(), TRACIK_INV_KIN_CHAIN_SOLVER_NAME);
  EXPECT_EQ(inv_kin->numJoints(), 7);
  EXPECT_EQ(inv_kin->getBaseLinkId(), BASE_LINK);
  ASSERT_EQ(inv_kin->getTipLinkIds().size(), 1);
  EXPECT_EQ(inv_kin->getTipLinkIds().front(), TIP_LINK);

  // A config without params leaves every solver parameter at its default
  const auto* trac_ik = dynamic_cast<const TracIKInvKinChain*>(inv_kin.get());
  ASSERT_TRUE(trac_ik != nullptr);
  EXPECT_DOUBLE_EQ(trac_ik->getMaxTime(), MAX_TIME);
  EXPECT_DOUBLE_EQ(trac_ik->getEpsilon(), EPSILON);
  EXPECT_EQ(trac_ik->getSolveType(), SOLVE_TYPE);
  expectTwistEq(trac_ik->getBounds(), BOUNDS);
}

TEST(TesseractKinematicsFactoryUnit, TracIKCreateWithParamsUnit)  // NOLINT
{
  const Fixture fixture;

  {  // Every value differs from its default, so a parameter the factory drops or misreads reads back as the default
    auto inv_kin = create(fixture, R"(base_link: base_link
tip_link: tool0
params:
  max_time: 0.5
  epsilon: 1e-4
  solve_type: Distance
  bounds: [0.01, 0.02, 0.03, 0.1, 0.2, 0.3])");
    ASSERT_TRUE(inv_kin != nullptr);

    const auto expect_params = [](const InverseKinematics& kin) {
      const auto* trac_ik = dynamic_cast<const TracIKInvKinChain*>(&kin);
      ASSERT_TRUE(trac_ik != nullptr);
      EXPECT_DOUBLE_EQ(trac_ik->getMaxTime(), 0.5);
      EXPECT_DOUBLE_EQ(trac_ik->getEpsilon(), 1e-4);
      EXPECT_EQ(trac_ik->getSolveType(), TRAC_IK::SolveType::Distance);
      expectTwistEq(trac_ik->getBounds(), KDL::Twist(KDL::Vector(0.01, 0.02, 0.03), KDL::Vector(0.1, 0.2, 0.3)));
    };
    expect_params(*inv_kin);

    // A clone carries the same parameters
    expect_params(*inv_kin->clone());
  }

  // Every documented solve type maps to its own TRAC_IK::SolveType
  const std::vector<std::pair<std::string, TRAC_IK::SolveType>> solve_types{
    { "Speed", TRAC_IK::SolveType::Speed },   { "Distance", TRAC_IK::SolveType::Distance },
    { "Manip1", TRAC_IK::SolveType::Manip1 }, { "Manip2", TRAC_IK::SolveType::Manip2 },
    { "Manip3", TRAC_IK::SolveType::Manip3 },
  };
  for (const auto& [name, type] : solve_types)
  {
    auto inv_kin = create(fixture, "base_link: base_link\ntip_link: tool0\nparams:\n  solve_type: " + name);
    const auto* trac_ik = dynamic_cast<const TracIKInvKinChain*>(inv_kin.get());
    ASSERT_TRUE(trac_ik != nullptr) << "solve_type: " << name;
    EXPECT_EQ(trac_ik->getSolveType(), type) << "solve_type: " << name;
  }
}

TEST(TesseractKinematicsFactoryUnit, TracIKBoundsFromConfigUnit)  // NOLINT
{
  // Bounds are enforced per axis, so a configured sequence that reaches the solver in order can be told apart from one
  // that does not: the target is out of reach along z alone and only the matching linear tolerance is opened up
  const Fixture fixture;
  const KDLFwdKinChain kdl_fwd_kin(*fixture.scene_graph, BASE_LINK, TIP_LINK);
  const ForwardKinematics& fwd_kin = kdl_fwd_kin;

  Eigen::Isometry3d target_pose = fwd_kin.calcFwdKin(Eigen::VectorXd::Zero(7)).at(TIP_LINK);
  target_pose.translation().z() += 0.03;

  const tesseract::common::LinkIdTransformMap input{ { TIP_LINK, target_pose } };
  const Eigen::VectorXd seed = Eigen::VectorXd::Zero(7);

  {  // Bounds default to zero, which leaves epsilon in charge and rejects every configuration
    auto inv_kin = create(fixture, R"(base_link: base_link
tip_link: tool0
params:
  max_time: 0.5)");
    ASSERT_TRUE(inv_kin != nullptr);
    EXPECT_TRUE(inv_kin->calcInvKin(input, seed).empty());
  }

  {  // Opening the linear z tolerance past the overreach admits the closest configuration
    auto inv_kin = create(fixture, R"(base_link: base_link
tip_link: tool0
params:
  max_time: 0.5
  bounds: [0.0, 0.0, 0.05, 0.0, 0.0, 0.0])");
    ASSERT_TRUE(inv_kin != nullptr);
    EXPECT_FALSE(inv_kin->calcInvKin(input, seed).empty());
  }
}

TEST(TesseractKinematicsFactoryUnit, TracIKCreateConfigFailuresUnit)  // NOLINT
{
  // create() validates against the schema before the factory sees the config, and reports a
  // rejection by throwing
  const Fixture fixture;

  const std::vector<std::pair<std::string, std::string>> invalid_configs{
    { "missing base_link", "tip_link: tool0" },
    { "missing tip_link", "base_link: base_link" },
    { "unknown solve type", "base_link: base_link\ntip_link: tool0\nparams:\n  solve_type: Bogus" },
    { "bounds not a full twist", "base_link: base_link\ntip_link: tool0\nparams:\n  bounds: [0.01, 0.02, 0.03]" },
    { "bounds element not a number", "base_link: base_link\ntip_link: tool0\nparams:\n  bounds: [a, b, c, d, e, f]" },
    { "wrong value type", "base_link: base_link\ntip_link: tool0\nparams:\n  max_time: not_a_number" },
  };

  for (const auto& [label, config] : invalid_configs)
  {
    EXPECT_THROW(create(fixture, config), tesseract::common::PropertyTreeValidationError) << label;  // NOLINT
  }
}

TEST(TesseractKinematicsFactoryUnit, TracIKCreateChainFailureUnit)  // NOLINT
{
  // A config that passes the schema but names links that do not form a chain fails in the solver
  // constructor. create() propagates that; the plugin loader catches it and returns null.
  const Fixture fixture;
  const std::string config = R"(base_link: does_not_exist
tip_link: tool0)";

  try
  {
    static_cast<void>(create(fixture, config));
    FAIL() << "Expected the solver constructor to throw";
  }
  catch (const tesseract::common::PropertyTreeValidationError& exception)
  {
    FAIL() << "Expected the config to pass the schema: " << exception.what();
  }
  catch (const std::exception&)
  {
    SUCCEED();
  }

  tesseract::common::PluginInfo plugin_info;
  plugin_info.class_name = "TracIKInvKinChainFactory";
  plugin_info.config = YAML::Load(config);

  KinematicsPluginFactory plugin_factory;
  plugin_factory.addSearchPath(std::string(PLUGIN_DIR));
  plugin_factory.addSearchLibrary("tesseract_trac_ik_trac-ik_factory");
  EXPECT_TRUE(plugin_factory.createInvKin(
                  TRACIK_INV_KIN_CHAIN_SOLVER_NAME, plugin_info, *fixture.scene_graph, fixture.scene_state) == nullptr);
}

TEST(TesseractKinematicsFactoryUnit, TracIKCreateAggregatesSchemaValidationErrorsUnit)  // NOLINT
{
  // One rejection reports every problem in the config, not only the first one found
  const Fixture fixture;
  const std::string config = R"(base_link: ""
params:
  bounds: [0.01, 0.02, 0.03]
unknown_key: true)";

  try
  {
    static_cast<void>(create(fixture, config));
    FAIL() << "Expected schema validation to fail";
  }
  catch (const tesseract::common::PropertyTreeValidationError& exception)
  {
    EXPECT_GE(exception.errors().size(), 4U);
    const std::string message = exception.what();
    for (const char* key : { "base_link", "tip_link", "bounds", "unknown_key" })
      EXPECT_NE(message.find(key), std::string::npos) << key << " missing from:\n" << message;
  }
}

TEST(TesseractKinematicsFactoryUnit, TracIKCreateImplRejectsUnknownSolveTypeUnit)  // NOLINT
{
  // An unknown solve type that reaches the factory unvalidated throws instead of picking a solver
  const Fixture fixture;
  const UnvalidatedTracIKInvKinChainFactory factory;
  const KinematicsPluginFactory plugin_factory;
  tesseract::common::PropertyTree config = factory.schema();
  const auto errors = config.applyConfig(YAML::Load(R"(base_link: base_link
tip_link: tool0
params:
  solve_type: Bogus)"));
  ASSERT_FALSE(errors.empty());

  const auto create_impl = [&] {
    return factory.createImpl(
        TRACIK_INV_KIN_CHAIN_SOLVER_NAME, *fixture.scene_graph, fixture.scene_state, plugin_factory, config);
  };
  EXPECT_THROW(create_impl(), std::runtime_error);  // NOLINT
}

TEST(TesseractKinematicsFactoryUnit, LoadTracIKKinematicsUnit)  // NOLINT
{
  // Exercises the configuration documented in the README: the factory is resolved by class name out of the built
  // plugin library
  const Fixture fixture;
  const std::string yaml_string = R"(kinematic_plugins:
  search_libraries:
    - tesseract_trac_ik_trac-ik_factory
  inv_kin_plugins:
    iiwa_manipulator:
      default: TracIKInvKinChain
      plugins:
        TracIKInvKinChain:
          class: TracIKInvKinChainFactory
          config:
            base_link: base_link
            tip_link: tool0
            params:
              max_time: 0.5
              solve_type: Distance)";

  const tesseract::common::GeneralResourceLocator locator;
  KinematicsPluginFactory factory(yaml_string, locator);
  factory.addSearchPath(std::string(PLUGIN_DIR));

  auto inv_kin =
      factory.createInvKin("iiwa_manipulator", "TracIKInvKinChain", *fixture.scene_graph, fixture.scene_state);
  ASSERT_TRUE(inv_kin != nullptr);
  EXPECT_EQ(inv_kin->getSolverName(), "TracIKInvKinChain");
  EXPECT_EQ(inv_kin->numJoints(), 7);
  EXPECT_EQ(inv_kin->getBaseLinkId(), BASE_LINK);

  // Unknown group and unknown solver both resolve to nothing
  EXPECT_TRUE(factory.createInvKin("does_not_exist", "TracIKInvKinChain", *fixture.scene_graph, fixture.scene_state) ==
              nullptr);
  EXPECT_TRUE(factory.createInvKin("iiwa_manipulator", "does_not_exist", *fixture.scene_graph, fixture.scene_state) ==
              nullptr);
}

TEST(TesseractKinematicsFactoryUnit, LoadTracIKKinematicsRejectsInvalidConfigUnit)  // NOLINT
{
  // The loader validates each plugin config against its registered schema when the config is
  // loaded, before any solver is requested
  const std::string yaml_string = R"(kinematic_plugins:
  search_paths:
    - )" + std::string(PLUGIN_DIR) +
                                  R"(
  search_libraries:
    - tesseract_trac_ik_trac-ik_factory
  inv_kin_plugins:
    iiwa_manipulator:
      default: TracIKInvKinChain
      plugins:
        TracIKInvKinChain:
          class: TracIKInvKinChainFactory
          config:
            base_link: base_link
            tip_link: tool0
            params:
              solve_type: Bogus)";

  const tesseract::common::GeneralResourceLocator locator;
  try
  {
    const KinematicsPluginFactory factory(yaml_string, locator);
    FAIL() << "Expected the config to be rejected at load";
  }
  catch (const std::runtime_error& exception)
  {
    const std::string message = exception.what();
    EXPECT_NE(message.find("solve_type"), std::string::npos) << message;
    EXPECT_NE(message.find("not in enum list"), std::string::npos) << message;
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
