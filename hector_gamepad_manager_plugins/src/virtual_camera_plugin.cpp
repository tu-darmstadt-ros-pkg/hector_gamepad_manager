#include "hector_gamepad_manager_plugins/virtual_camera_plugin.hpp"

// Pan tilt on axis
// swap on short button press
// recenter on hold
namespace hector_gamepad_manager_plugins
{

void VirtualCameraPlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  const std::string plugin_namespace = getPluginName();

  pan_speed_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".pan_speed", std::ref( pan_speed_ ), "Maximum pan speed in rad/s.",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value > 0.0; } ) );

  tilt_speed_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".tilt_speed", std::ref( tilt_speed_ ), "Maximum tilt speed in rad/s.",
      hector::ParameterOptions<double>().setRange( 0.01, 6.28, 0.01 ) );

  max_pan_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".max_pan", std::ref( max_pan_ ), "Maximum pan angle in rad.",
      hector::ParameterOptions<double>().setRange( -6.28, 6.28, 0.01 ) );

  min_pan_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".min_pan", std::ref( min_pan_ ), "Minimum pan angle in rad.",
      hector::ParameterOptions<double>().setRange( -6.28, 6.28, 0.01 ) );

  max_tilt_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".max_tilt", std::ref( max_tilt_ ), "Maximum tilt angle in rad.",
      hector::ParameterOptions<double>().setRange( -6.28, 6.28, 0.01 ) );

  min_tilt_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".min_tilt", std::ref( min_tilt_ ), "Minimum tilt angle in rad.",
      hector::ParameterOptions<double>().setRange( -6.28, 6.28, 0.01 ) );

  back_camera_topic_name_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".back_camera_topic_name", std::ref( back_camera_topic_name_ ),
      "ROS topic name for the back camera transform." );

  front_camera_topic_name_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".front_camera_topic_name", std::ref( front_camera_topic_name_ ),
      "ROS topic name for the front camera transform." );

  stick_hold_time_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".stick_hold_time", std::ref( stick_hold_time_ ),
      "Time in seconds to hold the stick to trigger recentering.",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value > 0.0; } ) );

  invert_y_axis_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".invert_y_axis", std::ref( invert_y_axis_ ),
      "Whether to invert the Y axis of the camera control. Default Y direction is downwards.",
      hector::ParameterOptions<bool>() );

  loadCameras();
  RCLCPP_INFO(
      node_->get_logger(),
      "Initialized VirtualCameraPlugin with back camera topic '%s' and front camera topic '%s'",
      back_camera_topic_name_.c_str(), front_camera_topic_name_.c_str() );
}

void VirtualCameraPlugin::handleAxis( const std::string &function, const std::string &id,
                                      const double value )
{
  // Ignore zero Inputs
  if ( std::abs( value ) <= 1e-3 ) {
    return;
  }

  double &pan = reverse_ ? back_transform_[2] : front_transform_[2];
  double &tilt = reverse_ ? back_transform_[1] : front_transform_[1];

  // value should always be between -1 and 1
  if ( function == "camera_pan" ) {
    auto gain = value * pan_speed_ * delta_t_;
    pan = std::clamp( pan + gain, min_pan_, max_pan_ );
    pose_changed_ = true;
  } else if ( function == "camera_tilt" ) {
    double direction;
    if ( invert_y_axis_ ) {
      direction = -1.0;
    } else {
      direction = 1.0;
    }
    auto gain = direction * value * tilt_speed_ * delta_t_;
    tilt = std::clamp( tilt + gain, min_tilt_, max_tilt_ );
    pose_changed_ = true;
  }
}

void VirtualCameraPlugin::handleHold( const std::string &function, const std::string &id )
{
  if ( function == "reset_camera" ) {
    stick_hold_timer_ += delta_t_;
  }
}

void VirtualCameraPlugin::handlePress( const std::string &function, const std::string &id )
{
  if ( function == "reset_camera" ) {
    stick_hold_timer_ = 0.0;
  }
}

void VirtualCameraPlugin::handleRelease( const std::string &function, const std::string &id )
{
  if ( function == "reset_camera" ) {
    if ( stick_hold_timer_ < stick_hold_time_ ) {
      // Currently unused till swap to virtual camera plugin is implemented
    } else {
      // Recenter camera
      if ( reverse_ ) {
        back_transform_ = { 0.0, 0.0, 0.0 };
        back_transform_publisher_->publish( geometry_msgs::msg::Transform() );
      } else {
        front_transform_ = { 0.0, 0.0, 0.0 };
        front_transform_publisher_->publish( geometry_msgs::msg::Transform() );
      }
    }
  }
}

void VirtualCameraPlugin::loadCameras()
{
  std::string back_topic_name = node_->get_effective_namespace() + "/" + back_camera_topic_name_;
  std::string front_topic_name = node_->get_effective_namespace() + "/" + front_camera_topic_name_;

  front_transform_publisher_ =
      node_->create_publisher<geometry_msgs::msg::Transform>( front_topic_name, 1 );
  back_transform_publisher_ =
      node_->create_publisher<geometry_msgs::msg::Transform>( back_topic_name, 1 );
}

void VirtualCameraPlugin::update()
{
  if ( !active_ || !pose_changed_ ) {
    return;
  }

  reverse_ = blackboard_->value_or<bool>( "invert_steering", false );

  double pan = reverse_ ? back_transform_[2] : front_transform_[2];
  double tilt = reverse_ ? back_transform_[1] : front_transform_[1];
  Eigen::Isometry3d orientation =
      Eigen::Isometry3d( Eigen::AngleAxisd( pan, Eigen::Vector3d::UnitZ() ) *
                         Eigen::AngleAxisd( tilt, Eigen::Vector3d::UnitY() ) *
                         Eigen::AngleAxisd( 0.0, Eigen::Vector3d::UnitX() ) );
  geometry_msgs::msg::Transform transform_msg = tf2::eigenToTransform( orientation ).transform;
  if ( reverse_ ) {
    back_transform_publisher_->publish( transform_msg );
  } else {
    front_transform_publisher_->publish( transform_msg );
  }
  pose_changed_ = false;
}

void VirtualCameraPlugin::activate() { active_ = true; }

void VirtualCameraPlugin::deactivate() { active_ = false; }

} // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::VirtualCameraPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )
