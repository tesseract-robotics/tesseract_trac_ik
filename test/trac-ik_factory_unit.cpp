#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_trac_ik/trac-ik/trac-ik_factory.h>
#include <tesseract_trac_ik/trac-ik/trac-ik_inv_kin_chain.h>

#include <tesseract/kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract/kinematics/kinematics_plugin_factory.h>
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

/**
 * @brief Create through the factory directly, bypassing plugin loading
 *
 * TracIKInvKinChainFactory::create() is only accessible through the InvKinFactory interface.
 */
std::unique_ptr<InverseKinematics> create(const Fixture& fixture, const std::string& config)
{
  const TracIKInvKinChainFactory factory;
  const InvKinFactory& inv_kin_factory = factory;
  const KinematicsPluginFactory plugin_factory;
  return inv_kin_factory.create(
      TRACIK_INV_KIN_CHAIN_SOLVER_NAME, *fixture.scene_graph, fixture.scene_state, plugin_factory, YAML::Load(config));
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
}

TEST(TesseractKinematicsFactoryUnit, TracIKCreateWithParamsUnit)  // NOLINT
{
  const Fixture fixture;

  {  // All parameters supplied
    auto inv_kin = create(fixture, R"(base_link: base_link
tip_link: tool0
params:
  max_time: 0.5
  epsilon: 1e-4
  solve_type: Distance
  bounds: [0.01, 0.02, 0.03, 0.1, 0.2, 0.3])");
    EXPECT_TRUE(inv_kin != nullptr);
  }

  // Every documented solve type is accepted
  const std::vector<std::string> solve_types{ "Speed", "Distance", "Manip1", "Manip2" };
  for (const std::string& solve_type : solve_types)
  {
    auto inv_kin = create(fixture, R"(base_link: base_link
tip_link: tool0
params:
  solve_type: )" + solve_type);
    EXPECT_TRUE(inv_kin != nullptr) << "solve_type: " << solve_type;
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
  const Fixture fixture;

  {  // Missing base_link
    EXPECT_TRUE(create(fixture, "tip_link: tool0") == nullptr);
  }

  {  // Missing tip_link
    EXPECT_TRUE(create(fixture, "base_link: base_link") == nullptr);
  }

  {  // Unknown solve type
    EXPECT_TRUE(create(fixture, R"(base_link: base_link
tip_link: tool0
params:
  solve_type: Bogus)") == nullptr);
  }

  {  // Bounds must be a full twist
    EXPECT_TRUE(create(fixture, R"(base_link: base_link
tip_link: tool0
params:
  bounds: [0.01, 0.02, 0.03])") == nullptr);
  }

  {  // Wrong value type
    EXPECT_TRUE(create(fixture, R"(base_link: base_link
tip_link: tool0
params:
  max_time: not_a_number)") == nullptr);
  }
}

TEST(TesseractKinematicsFactoryUnit, TracIKCreateChainFailureUnit)  // NOLINT
{
  // A config that parses but names links that do not form a chain fails in the constructor, which the factory has to
  // report the same way as a bad config: logged and null, never an exception through create()
  const Fixture fixture;
  const std::string config = R"(base_link: does_not_exist
tip_link: tool0)";

  EXPECT_TRUE(create(fixture, config) == nullptr);

  tesseract::common::PluginInfo plugin_info;
  plugin_info.class_name = "TracIKInvKinChainFactory";
  plugin_info.config = YAML::Load(config);

  KinematicsPluginFactory plugin_factory;
  plugin_factory.addSearchPath(std::string(PLUGIN_DIR));
  plugin_factory.addSearchLibrary("tesseract_trac_ik_trac-ik_factory");
  EXPECT_TRUE(plugin_factory.createInvKin(
                  TRACIK_INV_KIN_CHAIN_SOLVER_NAME, plugin_info, *fixture.scene_graph, fixture.scene_state) == nullptr);
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
