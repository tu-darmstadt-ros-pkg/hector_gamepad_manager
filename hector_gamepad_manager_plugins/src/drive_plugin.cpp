#include "hector_gamepad_manager_plugins/drive_plugin.hpp"

namespace hector_gamepad_manager_plugins
{

void DrivePlugin::initialize( const rclcpp::Node::SharedPtr &node, const std::string &robot_name )
{
  node_ = node;
  plugin_namespace_ = "drive_plugin";

  node_->declare_parameters<double>( plugin_namespace_, { { "max_linear_speed", 1.0 },
                                                          { "max_angular_speed", 1.0 },
                                                          { "slow_factor", 0.5 },
                                                          { "normal_factor", 0.75 },
                                                          { "fast_factor", 1.0 } } );
  node_->declare_parameters<std::string>( plugin_namespace_, { { "cmd_vel_topic", "cmd_vel" } } );

  max_linear_speed_ = node_->get_parameter( plugin_namespace_ + ".max_linear_speed" ).as_double();

  max_angular_speed_ = node_->get_parameter( plugin_namespace_ + ".max_angular_speed" ).as_double();

  slow_factor_ = node_->get_parameter( plugin_namespace_ + ".slow_factor" ).as_double();

  normal_factor_ = node_->get_parameter( plugin_namespace_ + ".normal_factor" ).as_double();

  fast_factor_ = node_->get_parameter( plugin_namespace_ + ".fast_factor" ).as_double();

  cmd_vel_topic = node_->get_parameter( plugin_namespace_ + ".cmd_vel_topic" ).as_string();

  fast_mode_active_ = false;
  slow_mode_active_ = false;

  drive_command_publisher_ =
      node_->create_publisher<geometry_msgs::msg::TwistStamped>( "/" + robot_name + "/" + cmd_vel_topic, 1 );
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

  drive_command_.twist.linear.x = drive_value_ * max_linear_speed_ * speed_factor;
  drive_command_.twist.angular.z = steer_value_ * max_angular_speed_ * speed_factor;

  drive_command_.header.stamp = node_->now();
  drive_command_publisher_->publish( drive_command_ );
}

void DrivePlugin::switchControlledRobot( const std::string &robot_name )
{
  drive_command_publisher_ =
      node_->create_publisher<geometry_msgs::msg::TwistStamped>( "/" + robot_name + "/" + cmd_vel_topic, 1 );
}

void DrivePlugin::activate() { active_ = true; }

void DrivePlugin::deactivate()
{
  active_ = false;

  // Bring the robot to a stop
  drive_command_.twist.linear.x = 0.0;
  drive_command_.twist.angular.z = 0.0;

  drive_command_.header.stamp = node_->now();
  drive_command_publisher_->publish( drive_command_ );
}
} // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::DrivePlugin,
                        hector_gamepad_manager::GamepadFunctionPlugin )
