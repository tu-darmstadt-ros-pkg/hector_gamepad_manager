#include "hector_gamepad_manager_plugins/manipulation_plugin.hpp"

namespace hector_gamepad_manager_plugins
{

void ManipulationPlugin::initialize( const rclcpp::Node::SharedPtr &node, const bool active )
{
  node_ = node;
  active_ = active;

  plugin_namespace_ = "manipulation_plugin";

  node_->declare_parameters<double>( plugin_namespace_, { { "max_eef_linear_speed", 1.0 },
                                                          { "max_eef_angular_speed", 1.0 },
                                                          { "max_drive_linear_speed", 1.0 },
                                                          { "max_drive_angular_speed", 1.0 },
                                                          { "max_gripper_speed", 1.0 } } );

  max_eef_linear_speed_ =
      node_->get_parameter( plugin_namespace_ + ".max_eef_linear_speed" ).as_double();

  max_eef_angular_speed_ =
      node_->get_parameter( plugin_namespace_ + ".max_eef_angular_speed" ).as_double();
  max_drive_linear_speed_ =
      node_->get_parameter( plugin_namespace_ + ".max_drive_linear_speed" ).as_double();
  max_drive_angular_speed_ =
      node_->get_parameter( plugin_namespace_ + ".max_drive_angular_speed" ).as_double();
  max_gripper_speed_ = node_->get_parameter( plugin_namespace_ + ".max_gripper_speed" ).as_double();

  eef_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>( "teleop/eef_cmd", 10 );
  drive_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>( "cmd_vel", 10 );
  gripper_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>( "teleop/gripper_cmd", 10 );
  hold_mode_client_ = node_->create_client<std_srvs::srv::SetBool>( "teleop/hold_mode" );
}

void ManipulationPlugin::handleButton( const std::string &function, const bool pressed )
{
  if ( !active_ ) {
    return;
  }

  if ( function == "hold_mode" ) { // TODO wait for merge request -> handle on press and release
    if ( hold_mode_active_ != pressed ) {
      hold_mode_change_requested_ = true;
    }
    hold_mode_active_ = pressed;
  } else if ( pressed ) { // TODO wait for merge request -> handle on press and release
    if ( function == "rotate_roll_clockwise" ) {
      rotate_roll_clockwise = max_eef_angular_speed_;
    } else if ( function == "rotate_roll_counter_clockwise" ) {
      rotate_roll_counter_clockwise = max_eef_angular_speed_;
    } else if ( function == "gripper_open" ) {
      gripper_cmd_.data = max_gripper_speed_;
    } else if ( function == "gripper_close" ) {
      gripper_cmd_.data = max_gripper_speed_;
    }
  }
}

void ManipulationPlugin::handleAxis( const std::string &function, const double value )
{
  if ( !active_ ) {
    return;
  }

  if ( function == "move_left_right" ) {
    move_left_right_ = value;
  } else if ( function == "move_up_down" ) {
    move_up_down_ = value;
  } else if ( function == "move_forward" ) {
    move_forward_ = value;
  } else if ( function == "move_backward" ) {
    move_backward_ = value;
  } else if ( function == "rotate_pitch" ) {
    rotate_pitch = value;
  } else if ( function == "rotate_yaw" ) {
    rotate_yaw = value;
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
  if ( hold_mode_active_ ) {
    // left joystick moves base, ignore all other axis and buttons
    drive_cmd_.twist.linear.x = move_up_down_ * max_drive_linear_speed_;
    drive_cmd_.twist.angular.z = move_left_right_ * max_drive_angular_speed_;
  } else {
    eef_cmd_.twist.linear.x = ( move_forward_ - move_backward_ ) * max_eef_linear_speed_;
    eef_cmd_.twist.linear.y = move_left_right_ * max_eef_linear_speed_;
    eef_cmd_.twist.linear.z = move_up_down_ * max_eef_linear_speed_;
    eef_cmd_.twist.angular.x =
        ( rotate_roll_clockwise - rotate_roll_counter_clockwise ) * max_eef_angular_speed_;
    eef_cmd_.twist.angular.y = rotate_pitch * max_eef_angular_speed_;
    eef_cmd_.twist.angular.z = rotate_yaw * max_eef_angular_speed_;
    gripper_cmd_.data = ( open_gripper - close_gripper ) * max_gripper_speed_;
  }

  eef_cmd_.header.stamp = node_->now();
  eef_cmd_pub_->publish( eef_cmd_ );
  gripper_cmd_pub_->publish( gripper_cmd_ );
  drive_cmd_.header.stamp = node_->now();
  drive_cmd_pub_->publish( drive_cmd_ );
}

void ManipulationPlugin::activate() { active_ = true; }

void ManipulationPlugin::deactivate()
{
  active_ = false;

  reset();
  eef_cmd_.header.stamp = node_->now();
  eef_cmd_pub_->publish( eef_cmd_ );
  gripper_cmd_pub_->publish( gripper_cmd_ );
  drive_cmd_.header.stamp = node_->now();
  drive_cmd_pub_->publish( drive_cmd_ );
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
  rotate_pitch = 0.0;
  rotate_yaw = 0.0;
  rotate_roll_clockwise = 0.0;
  rotate_roll_counter_clockwise = 0.0;
  open_gripper = 0.0;
  close_gripper = 0.0;
  eef_cmd_.twist = geometry_msgs::msg::Twist();
  drive_cmd_.twist = geometry_msgs::msg::Twist();
  gripper_cmd_.data = 0.0;
  hold_mode_change_requested_ = false;
}
} // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::ManipulationPlugin,
                        hector_gamepad_manager::GamepadFunctionPlugin )