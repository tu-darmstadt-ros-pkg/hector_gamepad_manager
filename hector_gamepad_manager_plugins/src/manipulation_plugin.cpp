#include "hector_gamepad_manager_plugins/manipulation_plugin.hpp"

namespace hector_gamepad_manager_plugins
{

void ManipulationPlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;

  std::string plugin_namespace = getPluginName();

  node_->declare_parameters<double>( plugin_namespace, { { "max_eef_linear_speed", 1.0 },
                                                          { "max_eef_angular_speed", 1.0 },
                                                          { "max_drive_linear_speed", 1.0 },
                                                          { "max_drive_angular_speed", 1.0 },
                                                          { "max_gripper_speed", 1.0 } } );

  max_eef_linear_speed_ =
      node_->get_parameter( plugin_namespace + ".max_eef_linear_speed" ).as_double();

  max_eef_angular_speed_ =
      node_->get_parameter( plugin_namespace + ".max_eef_angular_speed" ).as_double();
  max_drive_linear_speed_ =
      node_->get_parameter( plugin_namespace + ".max_drive_linear_speed" ).as_double();
  max_drive_angular_speed_ =
      node_->get_parameter( plugin_namespace + ".max_drive_angular_speed" ).as_double();
  max_gripper_speed_ = node_->get_parameter( plugin_namespace + ".max_gripper_speed" ).as_double();

  eef_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>( "teleop/eef_cmd", 10 );
  drive_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>( "cmd_vel", 10 );
  gripper_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>( "teleop/gripper_cmd", 10 );
  hold_mode_client_ = node_->create_client<std_srvs::srv::SetBool>( "teleop/hold_mode" );
}

std::string ManipulationPlugin::getPluginName() { return "manipulation_plugin"; }

void ManipulationPlugin::handlePress( const std::string &function )
{
  if ( function == "hold_mode" ) {
    hold_mode_change_requested_ = true;
    hold_mode_active_ = true;
  }
  else if ( function == "rotate_roll_clockwise" ) {
    rotate_roll_clockwise_ = 1;
  } else if ( function == "rotate_roll_counter_clockwise" ) {
    rotate_roll_counter_clockwise_ = -1;
  } else if ( function == "gripper_open" ) {
    open_gripper_ = 1;
  } else if ( function == "gripper_close" ) {
    close_gripper_ = -1;
  }
}

void ManipulationPlugin::handleRelease( const std::string &function )
{
  if ( function == "hold_mode" ) {
    hold_mode_change_requested_ = true;
    hold_mode_active_ = false;
  }
  else if ( function == "rotate_roll_clockwise" ) {
    rotate_roll_clockwise_ = 0.0;
  } else if ( function == "rotate_roll_counter_clockwise" ) {
    rotate_roll_counter_clockwise_ = 0.0;
  } else if ( function == "gripper_open" ) {
    open_gripper_ = 0.0;
  } else if ( function == "gripper_close" ) {
    close_gripper_ = 0.0;
  }
}


void ManipulationPlugin::handleAxis( const std::string &function, const double value )
{
  if ( function == "move_left_right" ) {
    move_left_right_ = value;
  } else if ( function == "move_up_down" ) {
    move_up_down_ = value;
  } else if ( function == "move_forward" ) {
    move_forward_ = value;
  } else if ( function == "move_backward" ) {
    move_backward_ = -value;
  } else if ( function == "rotate_pitch" ) {
    rotate_pitch_ = value;
  } else if ( function == "rotate_yaw" ) {
    rotate_yaw_ = value;
  }
}

void ManipulationPlugin::update()
{
  if ( !active_ ) {
    return;
  }

  if ( hold_mode_change_requested_ ) {
    const auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = hold_mode_active_;
    hold_mode_client_->async_send_request( request );
    // reset cmds when hold mode is toggled
    reset();
  }
  double cmd_vel_linear=0.0, cmd_vel_angular=0.0;
  if ( hold_mode_active_ ) {
    // left joystick moves base, ignore all other axis and buttons
    cmd_vel_linear = move_up_down_ * max_drive_linear_speed_;
    cmd_vel_angular = move_left_right_ * max_drive_angular_speed_;
  } else {
    eef_cmd_.twist.linear.x = ( move_forward_ + move_backward_ ) * max_eef_linear_speed_;
    eef_cmd_.twist.linear.y = move_left_right_ * max_eef_linear_speed_;
    eef_cmd_.twist.linear.z = move_up_down_ * max_eef_linear_speed_;
    eef_cmd_.twist.angular.x =
        ( rotate_roll_clockwise_ + rotate_roll_counter_clockwise_ ) * max_eef_angular_speed_;
    eef_cmd_.twist.angular.y = rotate_pitch_ * max_eef_angular_speed_;
    eef_cmd_.twist.angular.z = rotate_yaw_ * max_eef_angular_speed_;
    gripper_cmd_.data = ( open_gripper_ + close_gripper_ ) * max_gripper_speed_;
  }

  eef_cmd_.header.stamp = node_->now();
  eef_cmd_pub_->publish( eef_cmd_ );
  gripper_cmd_pub_->publish( gripper_cmd_ );
  sendDriveCommand( cmd_vel_linear, cmd_vel_angular );
}

void ManipulationPlugin::activate() { active_ = true; }

void ManipulationPlugin::deactivate()
{
  active_ = false;

  reset();
  eef_cmd_.header.stamp = node_->now();
  eef_cmd_pub_->publish( eef_cmd_ );
  gripper_cmd_pub_->publish( gripper_cmd_ );
  sendDriveCommand( 0,0 );
  // make sure hold mode is disabled when the plugin is deactivated
  if ( hold_mode_active_ ) {
    const auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    hold_mode_client_->async_send_request( request );
  }
}

void ManipulationPlugin::reset()
{
  move_left_right_ = 0.0;
  move_up_down_ = 0.0;
  move_forward_ = 0.0;
  move_backward_ = 0.0;
  rotate_pitch_ = 0.0;
  rotate_yaw_ = 0.0;
  rotate_roll_clockwise_ = 0.0;
  rotate_roll_counter_clockwise_ = 0.0;
  open_gripper_ = 0.0;
  close_gripper_ = 0.0;
  eef_cmd_.twist = geometry_msgs::msg::Twist();
  drive_cmd_.twist = geometry_msgs::msg::Twist();
  gripper_cmd_.data = 0.0;
  hold_mode_change_requested_ = false;
}

void ManipulationPlugin::sendDriveCommand( const double linear_speed, const double angular_speed )
{
  drive_cmd_.header.stamp = node_->now();
  drive_cmd_.twist.linear.x = linear_speed;
  drive_cmd_.twist.angular.z = angular_speed;
  const bool current_cmd_zero =
      drive_cmd_.twist.linear.x == 0.0 && drive_cmd_.twist.angular.z == 0.0;
  if ( !( current_cmd_zero && last_cmd_zero_ ) ) {
    drive_cmd_pub_->publish( drive_cmd_ );
  }
  last_cmd_zero_ = current_cmd_zero;
}

} // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::ManipulationPlugin,
                        hector_gamepad_manager::GamepadFunctionPlugin )