#include "hector_gamepad_manager_plugins/manipulation_plugin.hpp"

namespace hector_gamepad_manager_plugins
{

void ManipulationPlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  RCLCPP_INFO(node_->get_logger(), "ManipulationPlugin Node namespace: %s", node_->get_namespace());
  RCLCPP_INFO(node_->get_logger(), "ManipulationPlugin Node namespace: %s", node_->get_effective_namespace().c_str() );
  const auto plugin_namespace = getPluginName();

  node_->declare_parameter<double>( plugin_namespace + ".max_eef_linear_speed", 1.0 );
  node_->declare_parameter<double>( plugin_namespace + ".max_eef_angular_speed", 1.0 );
  node_->declare_parameter<double>( plugin_namespace + ".max_drive_linear_speed", 1.0 );
  node_->declare_parameter<double>( plugin_namespace + ".max_drive_angular_speed", 1.0 );
  node_->declare_parameter<double>( plugin_namespace + ".max_gripper_speed", 1.0 );

  node_->declare_parameter<std::string>( plugin_namespace + ".twist_controller_name",
                                         "moveit_twist_controller" );
  node_->declare_parameter<std::vector<std::string>>(
      plugin_namespace + ".stop_controllers",
      { "arm_trajectory_controller", "gripper_trajectory_controller" } );
  node_->declare_parameter<std::vector<std::string>>(plugin_namespace + ".pose_names",
                                                      { "front", "back", "folded", "door" });

  max_eef_linear_speed_ =
      node_->get_parameter( plugin_namespace + ".max_eef_linear_speed" ).as_double();
  max_eef_angular_speed_ =
      node_->get_parameter( plugin_namespace + ".max_eef_angular_speed" ).as_double();
  max_drive_linear_speed_ =
      node_->get_parameter( plugin_namespace + ".max_drive_linear_speed" ).as_double();
  max_drive_angular_speed_ =
      node_->get_parameter( plugin_namespace + ".max_drive_angular_speed" ).as_double();
  max_gripper_speed_ = node_->get_parameter( plugin_namespace + ".max_gripper_speed" ).as_double();

  twist_controller_name_ =
      node_->get_parameter( plugin_namespace + ".twist_controller_name" ).as_string();
  stop_controllers_ =
      node_->get_parameter( plugin_namespace + ".stop_controllers" ).as_string_array();
  const auto pose_names = node_->get_parameter( plugin_namespace + ".pose_names" ).as_string_array();
  for (size_t i=0; i<std::min(4uL,pose_names.size());i++) pose_names_[i] = pose_names[i];

  eef_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      twist_controller_name_ + "/eef_cmd", 10 );
  drive_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>( "cmd_vel", 10 );
  gripper_cmd_pub_ =
      node_->create_publisher<std_msgs::msg::Float64>( twist_controller_name_ + "/gripper_cmd", 10 );
  hold_mode_client_ =
      node_->create_client<std_srvs::srv::SetBool>( twist_controller_name_ + "/hold_mode" );
  controller_helper_.initialize( node, plugin_namespace );
  moveit_helper_.initialize( node_ );
}

std::string ManipulationPlugin::getPluginName() { return "manipulation_plugin"; }

void ManipulationPlugin::handlePress( const std::string &function )
{
  if ( function == "hold_mode" ) {
    hold_mode_change_requested_ = true;
    hold_mode_active_ = true;
  } else if ( function == "rotate_roll_clockwise" ) {
    rotate_roll_clockwise_ = 1;
  } else if ( function == "rotate_roll_counter_clockwise" ) {
    rotate_roll_counter_clockwise_ = -1;
  } else if ( function == "gripper_open" ) {
    open_gripper_ = 1;
  } else if ( function == "gripper_close" ) {
    close_gripper_ = -1;
  } else if ( function == "go_to_pose_1" ) {
    goal_pose_name_ = pose_names_[0];
  } else if ( function == "go_to_pose_2" ) {
    goal_pose_name_ = pose_names_[1];
  } else if ( function == "go_to_pose_3" ) {
    goal_pose_name_ = pose_names_[2];
  } else if ( function == "go_to_pose_4" ) {
    goal_pose_name_ = pose_names_[3];
  }
}

void ManipulationPlugin::handleRelease( const std::string &function )
{
  if ( function == "hold_mode" ) {
    hold_mode_change_requested_ = true;
    hold_mode_active_ = false;
  } else if ( function == "rotate_roll_clockwise" ) {
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
  double cmd_vel_linear = 0.0, cmd_vel_angular = 0.0;
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
  const bool is_zero_cmd = isZeroCmd();
  // make sure controllers are active
  if ( last_eef_cmd_zero_ && !is_zero_cmd ) {
    controller_helper_.switchControllers( { twist_controller_name_ }, stop_controllers_ );
    moveit_helper_.cancelGoal();
  }
  // avoid repeatedly sending zero commands
  if ( !( last_eef_cmd_zero_ && is_zero_cmd ) ) {
    eef_cmd_.header.stamp = node_->now();
    eef_cmd_pub_->publish( eef_cmd_ );
    gripper_cmd_pub_->publish( gripper_cmd_ );
  }
  if ( !hold_mode_active_ && last_eef_cmd_zero_ && is_zero_cmd ) {
    // check if a goal pose is set
    if ( !goal_pose_name_.empty() ) {
      sendNamedPoseGoal( goal_pose_name_ );
      goal_pose_name_ = "";
    }
  }
  last_eef_cmd_zero_ = is_zero_cmd;

  sendDriveCommand( cmd_vel_linear, cmd_vel_angular );
}

void ManipulationPlugin::activate() { active_ = true; }

void ManipulationPlugin::deactivate()
{
  active_ = false;

  reset();
  moveit_helper_.cancelGoal();
  eef_cmd_.header.stamp = node_->now();
  eef_cmd_pub_->publish( eef_cmd_ );
  gripper_cmd_pub_->publish( gripper_cmd_ );
  sendDriveCommand( 0, 0 );
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
  last_eef_cmd_zero_ = true;
  goal_pose_name_ = "";
}

void ManipulationPlugin::sendDriveCommand( const double linear_speed, const double angular_speed )
{
  drive_cmd_.header.stamp = node_->now();
  drive_cmd_.twist.linear.x = linear_speed;
  drive_cmd_.twist.angular.z = angular_speed;
  const bool current_cmd_zero = drive_cmd_.twist.linear.x == 0.0 && drive_cmd_.twist.angular.z == 0.0;
  if ( !( current_cmd_zero && last_drive_cmd_zero_ ) ) {
    drive_cmd_pub_->publish( drive_cmd_ );
  }
  last_drive_cmd_zero_ = current_cmd_zero;
}

bool ManipulationPlugin::isZeroCmd() const
{
  return move_left_right_ == 0.0 && move_up_down_ == 0.0 && move_forward_ == 0.0 &&
         move_backward_ == 0.0 && rotate_pitch_ == 0.0 && rotate_yaw_ == 0.0 &&
         rotate_roll_clockwise_ == 0.0 && rotate_roll_counter_clockwise_ == 0.0 &&
         open_gripper_ == 0.0 && close_gripper_ == 0.0;
}

void ManipulationPlugin::sendNamedPoseGoal( const std::string &pose_name )
{
  moveit_helper_.cancelGoal();
  controller_helper_.switchControllers(  stop_controllers_ ,{ twist_controller_name_ });
  moveit_helper_.sendNamedPoseGoal( pose_name );
}

} // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::ManipulationPlugin,
                        hector_gamepad_manager::GamepadFunctionPlugin )