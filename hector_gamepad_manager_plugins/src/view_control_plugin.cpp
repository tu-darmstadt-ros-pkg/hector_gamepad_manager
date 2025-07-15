//
// Created by aljoscha-schmidt on 7/13/25.
//
#include <hector_gamepad_manager_plugins/view_control_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace hector_gamepad_manager_plugins
{

/* --------------------------- initialisation ------------------------ */
void ViewControlPlugin::initialize( const rclcpp::Node::SharedPtr &node_robot_ns,
                                    const rclcpp::Node::SharedPtr &node_operator_ns )
{
  node_ = node_robot_ns;

  std::string rviz_node_name =
      node_->declare_parameter<std::string>( getPluginName() + ".rviz_node_name", "" );

  RCLCPP_WARN_STREAM( node_->get_logger(), getPluginName() << ".rviz_node_name: " << rviz_node_name );

  view_controller_pub_ =
      node_->create_publisher<hector_rviz_plugins_msgs::msg::RelativeViewControllerCmd>(
          rviz_node_name + "/hector_view_controller/relative_cmds", 10 );
  /* speed parameters */
  orbit_speed_ = node_->declare_parameter( getPluginName() + ".orbit_speed", 0.1 );
  zoom_speed_ = node_->declare_parameter( getPluginName() + ".zoom_speed", 0.1 );
  translate_speed_ = node_->declare_parameter( getPluginName() + ".translate_speed", 0.1 );

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
    RCLCPP_WARN_STREAM( node_->get_logger(), "Presetting " << f << " Not yet implemented" );
    /*const double d = preset_distance_;
    Point eye, focus;
    focus.x = focus.y = focus.z = 0.0; // origin in base_link

    if ( f == "preset_front" ) {
      eye.x = +d;
    } else if ( f == "preset_back" ) {
      eye.x = -d;
    } else if ( f == "preset_left" ) {
      eye.y = +d;
    } else if ( f == "preset_right" ) {
      eye.y = -d;
    }

    auto req = std::make_shared<MoveEyeAndFocus::Request>();
    req->header.frame_id = "base_link";
    req->eye = eye;
    req->focus = focus;
    move_eye_focus_cli_->async_send_request( req );

    focus_point_ = focus;*/
  }
}

void ViewControlPlugin::handleHold( const std::string &function )
{
  RCLCPP_INFO( node_->get_logger(), "Hadle hold for function %s", function.c_str() );
  if ( function == "move_up" )
    rviz_cmd_msg.translation.z = 0.5 * translate_speed_;
  else if ( function == "move_down" )
    rviz_cmd_msg.translation.z = -0.5 * translate_speed_;
}

void ViewControlPlugin::handleRelease( const std::string &f ) { }

/* ----------------------------- update ------------------------------ */
void ViewControlPlugin::update()
{
  if ( !active_ )
    return;
  view_controller_pub_->publish( rviz_cmd_msg );
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
  rviz_cmd_msg.stop_tracking = true;
  rviz_cmd_msg.switch_to_3d_mode = true;
}

/* ----------------------- plugin export ----------------------------- */
} // namespace hector_gamepad_manager_plugins
PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::ViewControlPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )
