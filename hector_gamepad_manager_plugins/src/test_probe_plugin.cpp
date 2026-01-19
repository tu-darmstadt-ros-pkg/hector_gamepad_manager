#include "hector_gamepad_manager_plugins/test_probe_plugin.hpp"

#include <iomanip>
#include <sstream>

namespace hector_gamepad_manager_plugins
{
void TestProbePlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  press_pub_ = node_->create_publisher<std_msgs::msg::String>( "test_probe/press", 10 );
  hold_pub_ = node_->create_publisher<std_msgs::msg::String>( "test_probe/hold", 10 );
  release_pub_ = node_->create_publisher<std_msgs::msg::String>( "test_probe/release", 10 );
  axis_pub_ = node_->create_publisher<std_msgs::msg::String>( "test_probe/axis", 10 );
}

void TestProbePlugin::handleAxis( const std::string &function, const std::string &id, double value )
{
  std::ostringstream value_stream;
  value_stream << std::fixed << std::setprecision( 3 ) << value;
  publishEvent( axis_pub_, "axis", function, id, value_stream.str() );
}

void TestProbePlugin::handlePress( const std::string &function, const std::string &id )
{
  publishEvent( press_pub_, "press", function, id );
}

void TestProbePlugin::handleHold( const std::string &function, const std::string &id )
{
  publishEvent( hold_pub_, "hold", function, id );
}

void TestProbePlugin::handleRelease( const std::string &function, const std::string &id )
{
  publishEvent( release_pub_, "release", function, id );
}

void TestProbePlugin::publishEvent( const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub,
                                    const std::string &event, const std::string &function,
                                    const std::string &id, const std::string &value )
{
  if ( !pub ) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = event + ":" + function + ":" + id;
  if ( !value.empty() ) {
    msg.data += ":" + value;
  }
  pub->publish( msg );
}
} // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::TestProbePlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )
