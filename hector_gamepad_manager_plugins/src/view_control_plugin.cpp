//
// Created by aljoscha-schmidt on 7/13/25.
//
#include <hector_gamepad_manager_plugins/view_control_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace hector_gamepad_manager_plugins
{

/* --------------------------- initialisation ------------------------ */
void ViewControlPlugin::initialize( const rclcpp::Node::SharedPtr &node_robot_ns )
{
  node_ = node_robot_ns;

  std::string rviz_node_name =
      node_->declare_parameter<std::string>( getPluginName() + ".rviz_node_name", "" );

  RCLCPP_WARN_STREAM( node_->get_logger(), getPluginName() << ".rviz_node_name: " << rviz_node_name );

  view_controller_pub_ =
      node_->create_publisher<hector_rviz_plugins_msgs::msg::RelativeViewControllerCmd>(
          rviz_node_name + "/hector_view_controller/relative_cmds", 10 );
  move_eye_and_focus_client_ = node_->create_client<hector_rviz_plugins_msgs::srv::MoveEyeAndFocus>(
      rviz_node_name + "/hector_view_controller/move_eye_and_focus" );
  track_frame_client_ = node_->create_client<hector_rviz_plugins_msgs::srv::TrackFrame>(
      rviz_node_name + "/hector_view_controller/set_tracked_frame" );
  /* speed parameters */
  orbit_speed_ = node_->declare_parameter( getPluginName() + ".orbit_speed", 0.1 );
  zoom_speed_ = node_->declare_parameter( getPluginName() + ".zoom_speed", 0.1 );
  translate_speed_ = node_->declare_parameter( getPluginName() + ".translate_speed", 0.1 );
  set_view_mode_client_ = node_->create_client<hector_rviz_plugins_msgs::srv::SetViewMode>(
      rviz_node_name + "/hector_view_controller/set_view_mode" );
  tracked_frame_ =
      node_->declare_parameter( getPluginName() + ".tracked_frame", std::string( "base_link" ) );
  preset_distance_ = node_->declare_parameter( getPluginName() + ".preset_direction", 2.2 );
  preset_eye_height_ = node_->declare_parameter( getPluginName() + ".preset_eye_height", 0.25 );
  preset_focus_height_ = node_->declare_parameter( getPluginName() + ".preset_focus_height", 0.55 );
  reset();
}

/* ------------------------------ input ------------------------------ */
void ViewControlPlugin::handleAxis( const std::string &f, double v )
{
  if ( f == "orbit_yaw" )
    rviz_cmd_msg.yaw_delta = v * orbit_speed_;
  else if ( f == "orbit_theta" )
    rviz_cmd_msg.theta_delta = v * orbit_speed_;
  else if ( f == "translate_x" )
    rviz_cmd_msg.translation.x = v * translate_speed_;
  else if ( f == "translate_y" )
    rviz_cmd_msg.translation.y = v * translate_speed_;
  else if ( f == "zoom_in" )
    rviz_cmd_msg.zoom_factor -= v * zoom_speed_; //  0…+1
  else if ( f == "zoom_out" )
    rviz_cmd_msg.zoom_factor += v * zoom_speed_; //  0…+1
  interacted_ = true;
}

void ViewControlPlugin::handlePress( const std::string &f )
{
  if ( f == "mode_2d" || f == "mode_3d" ) {
    rviz_cmd_msg.switch_to_3d_mode = ( f == "mode_3d" );
  } else if ( f == "track_base" ) {
    rviz_cmd_msg.stop_tracking = false;
  } else if ( f == "untrack" ) {
    rviz_cmd_msg.stop_tracking = true;
  }
  /* ------ presets (base_link frame) -------------------------------- */
  else if ( f.rfind( "preset_", 0 ) == 0 ) {
    if ( f == "preset_front" ) {
      preset_direction_ = PresetDirection::FRONT;
    } else if ( f == "preset_back" ) {
      preset_direction_ = PresetDirection::BACK;
    } else if ( f == "preset_left" ) {
      preset_direction_ = PresetDirection::LEFT;
    } else if ( f == "preset_right" ) {
      preset_direction_ = PresetDirection::RIGHT;
    }
  } else if ( f == "toggle_tracking" ) {
    tracking_active_ = !tracking_active_;
    if ( tracking_active_ )
      activate_tracking_ = true;
  } else if ( f == "activate_2d_mode" ) {
    activate_2d_mode_ = true;
  }
  interacted_ = true;
}

void ViewControlPlugin::handleHold( const std::string &function )
{
  if ( function == "move_up" )
    rviz_cmd_msg.translation.z = 0.5 * translate_speed_;
  else if ( function == "move_down" )
    rviz_cmd_msg.translation.z = -0.5 * translate_speed_;
  interacted_ = true;
}

/* ----------------------------- update ------------------------------ */
void ViewControlPlugin::update()
{
  if ( !active_ )
    return;
  // ******** Activate Tracking *******************************************
  if ( activate_tracking_ || preset_direction_ != PresetDirection::NONE ) {
    RCLCPP_INFO( node_->get_logger(), "Activating tracking for frame %s", tracked_frame_.c_str() );
    const auto req = std::make_shared<hector_rviz_plugins_msgs::srv::TrackFrame::Request>();
    req->frame = tracked_frame_;
    if ( track_frame_client_->wait_for_service( std::chrono::milliseconds( 50 ) ) )
      track_frame_client_->async_send_request( req );
    else
      RCLCPP_ERROR( node_->get_logger(), "Service %s not available",
                    track_frame_client_->get_service_name() );
    activate_tracking_ = false;
  }
  // ******** Preset Directions *******************************************
  if ( preset_direction_ == PresetDirection::NONE && interacted_ ) {
    view_controller_pub_->publish( rviz_cmd_msg );
  } else {
    const auto req = std::make_shared<hector_rviz_plugins_msgs::srv::MoveEyeAndFocus::Request>();
    req->header.frame_id = tracked_frame_;
    switch ( preset_direction_ ) {
    case PresetDirection::FRONT:
      req->eye.x = -preset_distance_;
      break;
    case PresetDirection::BACK:
      req->eye.x = preset_distance_;
      break;
    case PresetDirection::LEFT:
      req->eye.y = -preset_distance_;
      break;
    case PresetDirection::RIGHT:
      req->eye.y = preset_distance_;
      break;
    default:
      req->eye.x = -preset_distance_;
    }
    req->eye.z = preset_eye_height_;
    req->focus.z = preset_focus_height_;
    req->disable_animation = true;
    req->stop_tracking = !tracking_active_;
    if ( move_eye_and_focus_client_->wait_for_service( std::chrono::milliseconds( 50 ) ) )
      move_eye_and_focus_client_->async_send_request( req );
    else
      RCLCPP_WARN( node_->get_logger(), "Service %s not available",
                   move_eye_and_focus_client_->get_service_name() );
  }
  // ******** 2D MODE ********************************************************
  if ( activate_2d_mode_ ) {
    const auto req = std::make_shared<hector_rviz_plugins_msgs::srv::SetViewMode::Request>();
    req->mode.mode = 1;
    req->disable_animation = true;
    if ( set_view_mode_client_->wait_for_service( std::chrono::milliseconds( 50 ) ) )
      set_view_mode_client_->async_send_request( req );
    else
      RCLCPP_WARN( node_->get_logger(), "Service %s not available",
                   move_eye_and_focus_client_->get_service_name() );
  }

  reset();
}

void ViewControlPlugin::deactivate()
{
  active_ = false;
  reset();
}
void ViewControlPlugin::reset()
{
  // Reset cmds
  rviz_cmd_msg = hector_rviz_plugins_msgs::msg::RelativeViewControllerCmd();
  rviz_cmd_msg.zoom_factor = 1.0;
  rviz_cmd_msg.disable_animation = true;
  rviz_cmd_msg.stop_tracking = !tracking_active_;
  rviz_cmd_msg.switch_to_3d_mode = true;
  preset_direction_ = PresetDirection::NONE;
  interacted_ = false;
  activate_2d_mode_ = false;
}

/* ----------------------- plugin export ----------------------------- */
} // namespace hector_gamepad_manager_plugins
PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::ViewControlPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )
