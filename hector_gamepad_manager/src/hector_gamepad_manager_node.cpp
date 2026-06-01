#include "hector_gamepad_manager/hector_gamepad_manager.hpp"

#include <rclcpp/rclcpp.hpp>

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );

  using hector_gamepad_manager::HectorGamepadManager;

  // Plugin-param sets are now loaded per robot node inside the manager (each robot gets its own
  // node carrying the selected namespace + plugin-param overrides), so the base node needs no
  // pre-resolved overrides here.
  const auto node = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_node" );
  // A plugin whose parameter override type mismatches its declared type (e.g. an int where a
  // double is expected) fails to load and is reported by HectorGamepadManager; it does not
  // crash the node. See the type convention in config/plugin_params/<name>.yaml.
  auto hector_gamepad_manager = std::make_shared<HectorGamepadManager>( node );
  // The manager owns the executor (it adds the OCS node and every robot node to it).
  hector_gamepad_manager->spin();
  rclcpp::shutdown();
  return 0;
}
