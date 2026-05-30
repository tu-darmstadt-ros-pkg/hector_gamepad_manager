#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>

#include <hector_gamepad_manager/hector_gamepad_manager.hpp>
#include <hector_gamepad_manager/plugin_param_loader.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using hector_gamepad_manager::HectorGamepadManager;

namespace
{
std::string testConfigDir()
{
  return ( std::filesystem::path( __FILE__ ).parent_path() / "config" ).string();
}

// Build a manager node that starts in the DrivePlugin config with the given plugin-param set
// injected as parameter overrides (mirroring what the node executable does at startup).
std::shared_ptr<rclcpp::Node> makeNode( const std::string &plugin_params_name )
{
  std::vector<rclcpp::Parameter> overrides = {
      { "config_name", "manager_internal_drive_switches" },
      { "config_directory", testConfigDir() }, // absolute -> resolved against the source tree
      { "robot_namespace", "athena" },
      { "ocs_namespace", "ocs" },
      { "plugin_params", plugin_params_name },
  };
  const auto set = hector_gamepad_manager::loadPluginParamSet( testConfigDir(), plugin_params_name );
  overrides.insert( overrides.end(), set.begin(), set.end() );

  rclcpp::NodeOptions opts;
  opts.parameter_overrides( overrides );
  return std::make_shared<rclcpp::Node>( "plugin_params_test", opts );
}

constexpr char kSpeedParam[] = "drive_plugin.max_linear_speed";
} // namespace

// Proves the chain: YAML -> flattenPluginParams -> NodeOptions override -> plugin declare picks
// up the override value.
TEST( PluginParamsIntegration, OverrideReachesPluginParameter )
{
  const auto node = makeNode( "drive_a" );
  const auto manager = std::make_shared<HectorGamepadManager>( node );

  ASSERT_TRUE( node->has_parameter( kSpeedParam ) );
  EXPECT_DOUBLE_EQ( node->get_parameter( kSpeedParam ).as_double(), 1.0 );
}

// Snapshot / restore / reset semantics of applyPluginParamSet.
TEST( PluginParamsIntegration, SnapshotRestoreAndReset )
{
  const auto node = makeNode( "drive_a" );
  const auto manager = std::make_shared<HectorGamepadManager>( node );
  ASSERT_TRUE( node->has_parameter( kSpeedParam ) );
  ASSERT_DOUBLE_EQ( node->get_parameter( kSpeedParam ).as_double(), 1.0 );

  // Simulate a runtime change while set "drive_a" is active.
  node->set_parameter( rclcpp::Parameter( kSpeedParam, 9.0 ) );
  ASSERT_DOUBLE_EQ( node->get_parameter( kSpeedParam ).as_double(), 9.0 );

  // Switch to "drive_b": its YAML default applies, the "drive_a" runtime value is snapshotted.
  manager->applyPluginParamSet( "drive_b", false );
  EXPECT_DOUBLE_EQ( node->get_parameter( kSpeedParam ).as_double(), 2.0 );

  // Back to "drive_a" without reset: the snapshotted runtime value (9.0) is restored.
  manager->applyPluginParamSet( "drive_a", false );
  EXPECT_DOUBLE_EQ( node->get_parameter( kSpeedParam ).as_double(), 9.0 );

  // "drive_a" with reset: reload the YAML defaults (1.0).
  manager->applyPluginParamSet( "drive_a", true );
  EXPECT_DOUBLE_EQ( node->get_parameter( kSpeedParam ).as_double(), 1.0 );
}
