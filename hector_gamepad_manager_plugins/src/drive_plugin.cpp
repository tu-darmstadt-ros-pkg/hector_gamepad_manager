#include "hector_gamepad_manager_plugins/drive_plugin.hpp"

namespace hector_gamepad_manager_plugins
{


void DrivePlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  plugin_namespace_ = "drive_plugin";

  node_->declare_parameters<double>( plugin_namespace_, { { "max_linear_speed", 1.0 },
                                                          { "max_angular_speed", 1.0 },
                                                          { "slow_factor", 0.5 },
                                                          { "normal_factor", 0.75 },
                                                          { "fast_factor", 1.0 } } );

  max_linear_speed_ = node_->get_parameter( plugin_namespace_ + ".max_linear_speed" ).as_double();

  max_angular_speed_ = node_->get_parameter( plugin_namespace_ + ".max_angular_speed" ).as_double();

  slow_factor_ = node_->get_parameter( plugin_namespace_ + ".slow_factor" ).as_double();

  normal_factor_ = node_->get_parameter( plugin_namespace_ + ".normal_factor" ).as_double();

  fast_factor_ = node_->get_parameter( plugin_namespace_ + ".fast_factor" ).as_double();

  drive_command_publisher_ =
      node_->create_publisher<geometry_msgs::msg::TwistStamped>( "cmd_vel", 1 );
}

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
                        hector_gamepad_manager::GamepadFunctionPlugin )