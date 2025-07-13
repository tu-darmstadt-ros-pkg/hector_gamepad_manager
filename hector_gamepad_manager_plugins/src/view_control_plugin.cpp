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
  node_ = node_operator_ns;

  std::string rviz_node_name =
      node_->declare_parameter<std::string>( getPluginName() + ".rviz_node_name", "" );

  RCLCPP_WARN_STREAM( node_->get_logger(), getPluginName() << ".rviz_node_name: " << rviz_node_name );
  move_eye_cli_ =
      node_->create_client<MoveEye>( rviz_node_name + "/hector_view_controller/move_eye" );
  move_eye_focus_cli_ = node_->create_client<MoveEyeAndFocus>(
      rviz_node_name + "/hector_view_controller/move_eye_and_focus" );
  mode_cli_ =
      node_->create_client<SetViewMode>( rviz_node_name + "/hector_view_controller/set_view_mode" );
  track_cli_ = node_->create_client<TrackFrame>( rviz_node_name +
                                                 "/hector_view_controller/set_tracked_frame" );

  /* speed parameters */
  node_->declare_parameter( getPluginName() + ".orbit_speed", orbit_speed_ );
  node_->declare_parameter( getPluginName() + ".zoom_speed", zoom_speed_ );
  node_->declare_parameter( getPluginName() + ".translate_speed", translate_speed_ );
  node_->get_parameter( getPluginName() + ".orbit_speed", orbit_speed_ );
  node_->get_parameter( getPluginName() + ".zoom_speed", zoom_speed_ );
  node_->get_parameter( getPluginName() + ".translate_speed", translate_speed_ );
}

/* ------------------------------ input ------------------------------ */
void ViewControlPlugin::handleAxis( const std::string &f, double v )
{
  if ( f == "orbit_yaw" )
    orbit_yaw_ = v;
  else if ( f == "orbit_theta" )
    orbit_theta_ = v;
  else if ( f == "translate_x" )
    translate_x_ = v;
  else if ( f == "translate_y" )
    translate_y_ = v;
  else if ( f == "zoom_in" )
    zoom_in_ = v; //  0…+1
  else if ( f == "zoom_out" )
    zoom_out_ = v; //  0…+1
}

void ViewControlPlugin::handlePress( const std::string &f )
{
  if ( f == "move_up" )
    move_z_dir_ = +1;
  else if ( f == "move_down" )
    move_z_dir_ = -1;
  else if ( f == "mode_2d" || f == "mode_3d" ) {
    auto req = std::make_shared<SetViewMode::Request>();
    req->mode.mode = ( f == "mode_2d" ) ? hector_rviz_plugins_msgs::msg::ViewMode::MODE_2D
                                        : hector_rviz_plugins_msgs::msg::ViewMode::MODE_3D;
    mode_cli_->async_send_request( req );
  } else if ( f == "track_base" ) {
    auto req = std::make_shared<TrackFrame::Request>();
    req->frame = "base_link";
    track_cli_->async_send_request( req );
    tracking_on_ = true;
  } else if ( f == "untrack" ) {
    auto req = std::make_shared<TrackFrame::Request>();
    req->frame = "";
    track_cli_->async_send_request( req );
    tracking_on_ = false;
  }
  /* ------ presets (base_link frame) -------------------------------- */
  else if ( f.rfind( "preset_", 0 ) == 0 ) {
    const double d = preset_distance_;
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

    orbit_vector_ = Eigen::Vector3d( eye.x, eye.y, eye.z );
    focus_point_ = focus;
  }
}

void ViewControlPlugin::handleRelease( const std::string &f )
{
  if ( f == "move_up" || f == "move_down" )
    move_z_dir_ = 0;
}

/* ----------------------------- update ------------------------------ */
void ViewControlPlugin::update()
{
  if ( !active_ )
    return;
  if ( skip_x_ > counter_ ) {
    counter_++;
    return; // skip some frames to avoid flooding the service calls
  }
  RCLCPP_INFO_STREAM( node_->get_logger(), "ViewControlPlugin::update(): orbit_yaw_ = "
                                           << orbit_yaw_ << ", orbit_theta_ = " << orbit_theta_
                                           << ", translate_x_ = " << translate_x_
                                           << ", translate_y_ = " << translate_y_
                                           << ", zoom_in_ = " << zoom_in_
                                           << ", zoom_out_ = " << zoom_out_
                                           << ", move_z_dir_ = " << move_z_dir_ );
  const double dt = 1.0 / 60.0;

  /* ------ zoom ----------------------------------------------------- */
  /*double zoom_vel = ( zoom_in_ - zoom_out_ ) * zoom_speed_; // in m/s
  if ( std::fabs( zoom_vel ) > 1e-3 ) {
    Eigen::Vector3d dz = orbit_vector_.normalized() * zoom_vel * dt;
    translateWorld( dz );
  }

  /* ------ translate XY / Z ---------------------------------------- #1#
  if ( std::fabs( translate_x_ ) > 1e-3 || std::fabs( translate_y_ ) > 1e-3 || move_z_dir_ != 0 ) {
    Eigen::Vector3d x_axis( 1, 0, 0 ), y_axis( 0, 1, 0 ), z_axis( 0, 0, 1 );
    Eigen::Vector3d d = ( x_axis * -translate_x_   // left/right
                          + y_axis * translate_y_  // forward/back in world Y
                          + z_axis * move_z_dir_ ) // up/down
                        * translate_speed_ * dt;
    translateWorld( d );
  }*/

  /* ------ orbit ---------------------------------------------------- */
  if ( std::fabs( orbit_yaw_ ) > 1e-3 || std::fabs( orbit_theta_ ) > 1e-3 ) {
    orbit( orbit_yaw_ * orbit_speed_ * dt, orbit_theta_ * orbit_speed_ * dt );
  }
}

/* --------------------------- helpers ------------------------------- */
bool ViewControlPlugin::waitForServicesOnce()
{
  return true;
  const auto t = std::chrono::seconds( 0 );
  return move_eye_cli_->wait_for_service( t ) && move_eye_focus_cli_->wait_for_service( t ) &&
         mode_cli_->wait_for_service( t ) && track_cli_->wait_for_service( t );
}

void ViewControlPlugin::translateWorld( const Eigen::Vector3d &d )
{
  focus_point_.x += d.x();
  focus_point_.y += d.y();
  focus_point_.z += d.z();
  orbit_vector_ += d;

  Point eye;
  eye.x = focus_point_.x + orbit_vector_.x();
  eye.y = focus_point_.y + orbit_vector_.y();
  eye.z = focus_point_.z + orbit_vector_.z();

  auto req = std::make_shared<MoveEye::Request>();
  req->header.frame_id = "world";
  req->eye = eye;
  req->stop_tracking = !tracking_on_;
  move_eye_cli_->async_send_request( req );
}

void ViewControlPlugin::orbit( double d_yaw, double d_pitch )
{
  Eigen::AngleAxisd yaw( d_yaw, Eigen::Vector3d::UnitZ() );
  Eigen::Vector3d axis_x = Eigen::Vector3d::UnitZ().cross( eye ).normalized();
  Eigen::AngleAxisd pitch( d_pitch, axis_x );

  orbit_vector_ = yaw * pitch * orbit_vector_;

  Point eye;
  eye.x = focus_point_.x + orbit_vector_.x();
  eye.y = focus_point_.y + orbit_vector_.y();
  eye.z = focus_point_.z + orbit_vector_.z();

  auto req = std::make_shared<MoveEye::Request>();
  req->header.frame_id = "world";
  req->eye = eye;
  req->stop_tracking = !tracking_on_;
  move_eye_cli_->async_send_request( req );
}

void ViewControlPlugin::deactivate()
{
  active_ = false;

  /* reset all live inputs so we don't send stale commands when
     the plugin is re‑activated later */
  orbit_yaw_ = orbit_theta_ = 0.0;
  translate_x_ = translate_y_ = 0.0;
  zoom_in_ = zoom_out_ = 0.0;
  move_z_dir_ = 0;
}

/* ----------------------- plugin export ----------------------------- */
} // namespace hector_gamepad_manager_plugins
PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::ViewControlPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin)
