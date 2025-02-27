#include "hector_gamepad_manager/hector_gamepad_manager.hpp"

#include <rclcpp/rclcpp.hpp>

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  const auto node = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_node" );
  rclcpp::executors::SingleThreadedExecutor exe;
  auto hector_gamepad_manager =
      std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node );
  exe.add_node(node);
  exe.spin();
  rclcpp::shutdown();
  return 0;
}