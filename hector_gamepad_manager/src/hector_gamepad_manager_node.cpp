#include "hector_gamepad_manager/hector_gamepad_manager.hpp"
#include "hector_gamepad_manager/plugin_param_loader.hpp"

#include <rclcpp/rclcpp.hpp>

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );

  using hector_gamepad_manager::HectorGamepadManager;

  // The selected plugin-param set must be present as parameter overrides at node construction:
  // plugins read their parameters once, at declare time, and some (e.g. moveit start_controllers)
  // are required with no default. We therefore resolve which set to load *before* building the
  // real node. A short-lived bootstrap node picks up the same global args (launch-file overrides)
  // the real node would see; it is discarded immediately.
  std::string config_directory = HectorGamepadManager::DEFAULT_CONFIG_DIRECTORY;
  std::string plugin_params_name = HectorGamepadManager::DEFAULT_PLUGIN_PARAMS;
  {
    auto bootstrap = std::make_shared<rclcpp::Node>( "_hector_gamepad_manager_bootstrap" );
    config_directory =
        bootstrap->declare_parameter<std::string>( "config_directory", config_directory );
    plugin_params_name =
        bootstrap->declare_parameter<std::string>( "plugin_params", plugin_params_name );
  }

  rclcpp::NodeOptions options;
  const auto overrides =
      hector_gamepad_manager::loadPluginParamSet( config_directory, plugin_params_name );
  if ( !overrides.empty() )
    options.parameter_overrides( overrides );

  const auto node = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_node", options );
  // A plugin whose parameter override type mismatches its declared type (e.g. an int where a
  // double is expected) fails to load and is reported by HectorGamepadManager; it does not
  // crash the node. See the type convention in config/plugin_params/<name>.yaml.
  auto hector_gamepad_manager = std::make_shared<HectorGamepadManager>( node );
  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}
