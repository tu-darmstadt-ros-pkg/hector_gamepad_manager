#include "hector_gamepad_manager/hector_gamepad_manager.hpp"

#include <rclcpp/rclcpp.hpp>

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  const auto node = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_node" );
  auto hector_gamepad_manager =
      std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node );
  std::shared_ptr<rclcpp::Executor> executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node( node );
  executor->spin();
  rclcpp::shutdown();
  return 0;
}