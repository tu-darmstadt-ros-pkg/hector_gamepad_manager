#include "hector_gamepad_manager_plugins/drive_plugin.hpp"

namespace hector_gamepad_manager_plugins
{

void DrivePlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  const std::string plugin_namespace = getPluginName();

  max_linear_speed_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".max_linear_speed", std::ref( max_linear_speed_ ),
      "Maximum linear speed in m/s in Normal Mode.",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value > 0.0; } ) );

  max_angular_speed_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".max_angular_speed", std::ref( max_angular_speed_ ),
      "Maximum angular speed in rad/s in Normal Mode.",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value > 0.0; } ) );

  slow_factor_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".slow_factor", std::ref( slow_factor_ ),
      "Scaling factor for speed in Slow Mode.",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value > 0.0 && value < 1.0; } ) );

  fast_factor_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".fast_factor", std::ref( fast_factor_ ),
      "Scaling factor for speed in Fast Mode.",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value >= 1.0; } ) );

  drive_command_publisher_ =
      node_->create_publisher<geometry_msgs::msg::TwistStamped>( "cmd_vel", 1 );
}

std::string DrivePlugin::getPluginName() { return "drive_plugin"; }

void DrivePlugin::handleAxis( const std::string &function, const double value )
{
  if ( function == "drive" ) {
    drive_value_ = value;
  } else if ( function == "steer" ) {
    steer_value_ = value;
  }
}

void DrivePlugin::handlePress( const std::string &function )
{
  if ( function == "fast" ) {
    fast_mode_active_ = true;
  } else if ( function == "slow" ) {
    slow_mode_active_ = true;
  }
}

void DrivePlugin::handleRelease( const std::string &function )
{
  if ( function == "fast" ) {
    fast_mode_active_ = false;
  } else if ( function == "slow" ) {
    slow_mode_active_ = false;
  }
}

void DrivePlugin::update()
{
  if ( !active_ ) {
    return;
  }

  double speed_factor = normal_factor_;
  if ( slow_mode_active_ ) {
    speed_factor = slow_factor_;
  } else if ( fast_mode_active_ ) {
    speed_factor = fast_factor_;
  }

  sendDriveCommand( drive_value_ * max_linear_speed_ * speed_factor,
                    steer_value_ * max_angular_speed_ * speed_factor );
}

void DrivePlugin::activate()
{
  active_ = true;
  last_cmd_zero_ = false;
}

void DrivePlugin::deactivate()
{
  active_ = false;
  sendDriveCommand( 0, 0 );
}

void DrivePlugin::sendDriveCommand( double linear_speed, double angular_speed )
{
  drive_command_.header.stamp = node_->now();
  drive_command_.twist.linear.x = linear_speed;
  drive_command_.twist.angular.z = angular_speed;
  bool current_cmd_zero =
      drive_command_.twist.linear.x == 0.0 && drive_command_.twist.angular.z == 0.0;
  if ( !( current_cmd_zero && last_cmd_zero_ ) ) {
    drive_command_publisher_->publish( drive_command_ );
  }
  last_cmd_zero_ = current_cmd_zero;
}
} // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::DrivePlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )