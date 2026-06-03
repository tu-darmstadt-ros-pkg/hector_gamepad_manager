#include "hector_gamepad_manager/joy_satellite.hpp"

#include <rclcpp/rclcpp.hpp>

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<hector_gamepad_manager::JoySatelliteNode>() );
  rclcpp::shutdown();
  return 0;
}
