#include "hector_gamepad_manager_plugins/drive_plugin.hpp"

namespace hector_gamepad_manager_plugins
{

void DrivePlugin::initialize( const rclcpp::Node::SharedPtr &node, const bool active )
{
  node_ = node;
  active_ = active;

  node_->declare_parameter<std::string>( "drive_command_topic", "/cmd_vel" );
  drive_command_topic_ = node_->get_parameter( "drive_command_topic" ).as_string();

  node_->declare_parameter<double>( "max_linear_speed", 1.0 );
  max_linear_speed_ = node_->get_parameter( "max_linear_speed" ).as_double();

  node_->declare_parameter<double>( "max_angular_speed", 1.0 );
  max_angular_speed_ = node_->get_parameter( "max_angular_speed" ).as_double();

  node_->declare_parameter<double>( "slow_factor", 0.5 );
  slow_factor_ = node_->get_parameter( "slow_factor" ).as_double();

  node_->declare_parameter<double>( "normal_factor", 0.75 );
  normal_factor_ = node_->get_parameter( "normal_factor" ).as_double();

  node_->declare_parameter<double>( "fast_factor", 1.0 );
  fast_factor_ = node_->get_parameter( "fast_factor" ).as_double();

  drive_command_publisher_ =
      node_->create_publisher<geometry_msgs::msg::TwistStamped>( drive_command_topic_, 10 );
}

void DrivePlugin::handleButton( const std::string &function, const bool pressed )
{
  if ( !active_ ) {
    return;
  }

  if ( function == "fast" ) {
    fast_button_pressed_ = pressed;
  } else if ( function == "slow" ) {
    slow_button_pressed_ = pressed;
  }
}

void DrivePlugin::handleAxis( const std::string &function, const double value )
{
  if ( !active_ ) {
    return;
  }

  if ( function == "drive" ) {
    drive_value_ = value;
  } else if ( function == "steer" ) {
    steer_value_ = value;
  }
}

void DrivePlugin::update()
{
  if ( !active_ ) {
    return;
  }

  double speed_factor = normal_factor_;
  if ( fast_button_pressed_ ) {
    speed_factor = fast_factor_;
  } else if ( slow_button_pressed_ ) {
    speed_factor = slow_factor_;
  }

  drive_command_.twist.linear.x = drive_value_ * max_linear_speed_ * speed_factor;
  drive_command_.twist.angular.z = steer_value_ * max_angular_speed_ * speed_factor;

  drive_command_.header.stamp = node_->now();
  drive_command_publisher_->publish( drive_command_ );
}

void DrivePlugin::activate() { active_ = true; }

void DrivePlugin::deactivate()
{
  active_ = false;

  drive_command_.twist.linear.x = 0.0;
  drive_command_.twist.angular.z = 0.0;
  drive_command_publisher_->publish( drive_command_ );
}
} // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::DrivePlugin,
                        hector_gamepad_manager::GamepadFunctionPlugin )
