#include "hector_gamepad_manager/hector_gamepad_manager.hpp"

#include <rclcpp/rclcpp.hpp>

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  const std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>( "hector_gamepad_manager_node" );
  std::shared_ptr<hector_gamepad_manager::HectorGamepadManager> hector_gamepad_manager =
      std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node );
  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}
