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
      node, plugin_namespace + ".pan_speed", std::ref( pan_speeds_ ),
      "Maximum pan speed in rad/s.",
      hector::ParameterOptions<std::vector<double>>().onValidate(
          []( const auto &value ) { return value.size() > 0 && std::all_of( value.begin(), value.end(), []( const auto &v ) { return v > 0.0; } ); } ) );

  tilt_speed_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".tilt_speed", std::ref( tilt_speeds_ ),
      "Maximum tilt speed in rad/s.",
      hector::ParameterOptions<std::vector<double>>().onValidate(
          []( const auto &value ) { return value.size() > 0 && std::all_of( value.begin(), value.end(), []( const auto &v ) { return v > 0.0; } ); } ) );

  max_pan_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".max_pan", std::ref( max_pans_ ), "Maximum pan angle in rad.",
      hector::ParameterOptions<std::vector<double>>());

  min_pan_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".min_pan", std::ref( min_pans_ ), "Minimum pan angle in rad.",
      hector::ParameterOptions<std::vector<double>>());

  max_tilt_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".max_tilt", std::ref( max_tilts_ ), "Maximum tilt angle in rad.",
      hector::ParameterOptions<std::vector<double>>());

  min_tilt_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".min_tilt", std::ref( min_tilts_ ), "Minimum tilt angle in rad.",
      hector::ParameterOptions<std::vector<double>>());
  
  camera_change_publisher_ =
      node->create_publisher<std_msgs::msg::String>( plugin_namespace + "/active_camera", 1);

  node_names_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".node_names", std::ref( node_names_ ), "List of camera node names.",
      hector::ParameterOptions<std::vector<std::string>>().onValidate(
          []( const auto &value ) {
            return value.size() > 0;
          } ).onUpdate(
          [this]( const auto &value ) {
              valid_subscription_ = false; // Reset to first camera when camera list changes
              loadCamera( 0 );
            } ) );

  camera_names_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".camera_names", std::ref( camera_names_ ), "List of camera names corresponding to the node names.",
      hector::ParameterOptions<std::vector<std::string>>().onUpdate(
          [this]( const auto &value ) {
            if ( value.size() != node_names_.size() ) {
              RCLCPP_WARN( node_->get_logger(), "Camera names list size does not match node names list size." );
            }
          } ));

  stick_hold_time_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".stick_hold_time", std::ref( stick_hold_time_ ), "Time in seconds to hold the stick to trigger recentering.",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value > 0.0; } ) );

  base_tilts_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".base_tilt", std::ref( base_tilts_ ), "Base tilt angles for each camera in radians.",
      hector::ParameterOptions<std::vector<double>>().onUpdate(
          [this]( const auto &value ) {
            if ( value.size() != node_names_.size() ) {
              RCLCPP_WARN( node_->get_logger(), "Base tilts list size does not match node names list size." );
            }
          } ) );
  
  base_pans_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".base_pan", std::ref( base_pans_ ), "Base pan angles for each camera in radians.",
      hector::ParameterOptions<std::vector<double>>().onUpdate(
          [this]( const auto &value ) {
            if ( value.size() != node_names_.size() ) {
              RCLCPP_WARN( node_->get_logger(), "Base pans list size does not match node names list size." );
            }
          } ) );
  base_rolls_param_sub_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".base_roll", std::ref( base_rolls_ ), "Base roll angles for each camera in radians.",
      hector::ParameterOptions<std::vector<double>>().onUpdate(
          [this]( const auto &value ) {
            if ( value.size() != node_names_.size() ) {
              RCLCPP_WARN( node_->get_logger(), "Base rolls list size does not match node names list size." );
            }
          } ) );

  loadCamera( 0 );
  RCLCPP_INFO( node_->get_logger(), "Virtual camera plugin loaded" );
}


void VirtualCameraPlugin::handleAxis( const std::string &function, const std::string &id, const double value )
{
  // Ignore zero Inputs
  if ( std::abs(value) <= 1e-3 ) {
    return;
  }
  // value should always be between -1 and 1
  if ( function == "camera_pan" ) {
    auto gain = value * pan_speed_ * delta_t_;
    RCLCPP_INFO( node_->get_logger(), "Handling camera pan with input value: %f, gain: %f", value, gain );
    current_pan_ = std::clamp( current_pan_ + 
      gain,
      min_pan_, max_pan_ );
    RCLCPP_INFO( node_->get_logger(), "Handling camera pan after: %f", current_pan_ );
    pose_changed_ = true;
  } else if ( function == "camera_tilt" ) {
    current_tilt_ = std::clamp( current_tilt_ + 
      value * tilt_speed_ * delta_t_, min_tilt_, max_tilt_);
    pose_changed_ = true;
  }
}


void VirtualCameraPlugin::handleHold( const std::string &function, const std::string &id )
{
  if ( function == "swap_camera" ) {
    stick_hold_timer_ += delta_t_;
  }
}


void VirtualCameraPlugin::handlePress( const std::string &function, const std::string &id )
{
  if ( function == "swap_camera" ) {
    stick_hold_timer_ = 0.0; 
  }
}


void VirtualCameraPlugin::handleRelease( const std::string &function, const std::string &id )
{
  if ( function == "swap_camera" ) {
    if ( stick_hold_timer_ < stick_hold_time_ ) {
      auto camera_idx = (active_camera_ + 1) % node_names_.size();
      loadCamera( camera_idx );
    } else {
      // Recenter camera
      std::string camera_param = node_names_[active_camera_];
      current_orientation_ = base_transform_;
      current_pan_ = 0.0;
      current_tilt_ = 0.0;
      param_client_->set_parameters({ rclcpp::Parameter("pose", base_pose_) });
    }
  }
}



/*
* Changes the active camera by updating the parameter client to point to the new camera's parameter namespace and retrieving the new camera's pose parameters.
*/
void VirtualCameraPlugin::loadCamera( const uint &camera_idx )
{
  std::string node_name = node_names_[camera_idx];
  // check if topic exists
    std::string topic_name = node_->get_effective_namespace() + "/" + node_name;
  param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(node_, node_->get_effective_namespace() + "/" + node_name);
  
  // TODO check if necessary to wait for service to be available or if we can just try to get parameters and catch the exception if it fails
  if ( !param_client_->wait_for_service( std::chrono::seconds( 5 ) ) ) {
    RCLCPP_ERROR( node_->get_logger(), "Parameter service not available for camera node: %s", node_name.c_str() );
    valid_subscription_ = false;
    active_camera_ = camera_idx;
    return;
  }
  
  auto future = param_client_->get_parameters( {"pose"}, 
    [this, camera_idx](std::shared_future<std::vector<rclcpp::Parameter>> f) {
        try {
            const auto & params = f.get();
            this->poseCallback(params, camera_idx);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Failed to get parameters: %s", e.what());
        }
    });
}

void VirtualCameraPlugin::poseCallback( const std::vector<rclcpp::Parameter> &parameters, const uint &active_camera )
{
  if ( active_camera_ == active_camera && valid_subscription_ ) {
    RCLCPP_WARN( node_->get_logger(), "Received pose parameters for already active camera. Ignoring." );
    return; 
  }

  active_camera_ = active_camera;
  std::vector<double> pose = parameters[0].as_double_array();

  double base_tilt = 0.0;
  if ( active_camera_ >= base_tilts_.size() ) {
    RCLCPP_WARN( node_->get_logger(), "Active camera index exceeds base tilts list size. Using default base tilt." );
    base_tilt = 0.0;
  } else {
    base_tilt = base_tilts_[active_camera];
  }

  double base_pan = 0.0;
  if ( active_camera_ >= base_pans_.size() ) {
    RCLCPP_WARN( node_->get_logger(), "Active camera index exceeds base pans list size. Using default base pan." );
    base_pan = 0.0;
  } else {
    base_pan = base_pans_[active_camera];
  }

  double base_roll = 0.0;
  if ( active_camera_ >= base_rolls_.size() ) {
    RCLCPP_WARN( node_->get_logger(), "Active camera index exceeds base rolls list size. Using default base roll." );
    base_roll = 0.0;
  } else {
    base_roll = base_rolls_[active_camera];
  }

  base_pose_ = {0.0, 0.0, 0.0, base_roll, base_tilt, base_pan};

  if ( pose.size() != 6 ) {
    RCLCPP_WARN( node_->get_logger(), "Camera pose parameter does not have 6 elements. Resetting to default." );
    pose = base_pose_;
  }

  // Get last transform
  current_orientation_ = Eigen::Affine3d::Identity();
  current_orientation_.rotate( Eigen::AngleAxisd( pose[5], Eigen::Vector3d::UnitZ() ) ); // yaw
  current_orientation_.rotate( Eigen::AngleAxisd( pose[4], Eigen::Vector3d::UnitY() ) ); // pitch
  current_orientation_.rotate( Eigen::AngleAxisd( pose[3], Eigen::Vector3d::UnitX() ) ); // roll
  
  current_orientation_.translation() = Eigen::Vector3d( pose[0], pose[1], pose[2] );

  // Get base transform
  base_transform_ = Eigen::Affine3d::Identity();
  base_transform_.rotate( Eigen::AngleAxisd( base_pan, Eigen::Vector3d::UnitZ() ) ); // yaw
  base_transform_.rotate( Eigen::AngleAxisd( base_tilt, Eigen::Vector3d::UnitY() ) ); // pitch
  base_transform_.rotate( Eigen::AngleAxisd( base_roll, Eigen::Vector3d::UnitX() ) ); // roll

  current_orientation_ = base_transform_.inverse() * current_orientation_; // Remove base transform from current orientation
  auto euler_angles = current_orientation_.rotation().eulerAngles(2, 1, 0);
  current_pan_ = euler_angles[0];
  current_tilt_ = euler_angles[1];
  

  if ( active_camera_ >= max_pans_.size() ) {
    RCLCPP_WARN( node_->get_logger(), "Active camera index exceeds max pans list size. Using default max pan." );
    max_pan_ = 0.0;
  } else {
    max_pan_ = max_pans_[active_camera];
  }

  if ( active_camera_ >= min_pans_.size() ) {
    RCLCPP_WARN( node_->get_logger(), "Active camera index exceeds min pans list size. Using default min pan." );
    min_pan_ = 0.0;
  } else {
    min_pan_ = min_pans_[active_camera];
  }

  if ( active_camera_ >= max_tilts_.size() ) {
    RCLCPP_WARN( node_->get_logger(), "Active camera index exceeds max tilts list size. Using default max tilt." );
    max_tilt_ = 0.0;
  } else {
    max_tilt_ = max_tilts_[active_camera];
  }

  if ( active_camera_ >= min_tilts_.size() ) {
    RCLCPP_WARN( node_->get_logger(), "Active camera index exceeds min tilts list size. Using default min tilt." );
    min_tilt_ = 0.0;
  } else {
    min_tilt_ = min_tilts_[active_camera];
  }

  if ( active_camera_ >= pan_speeds_.size() ) {
    RCLCPP_WARN( node_->get_logger(), "Active camera index exceeds max pan speeds list size. Using default max pan speed." );
    pan_speed_ = 0.5;
  } else {
    pan_speed_ = pan_speeds_[active_camera];
  }

  if ( active_camera_ >= tilt_speeds_.size() ) {
    RCLCPP_WARN( node_->get_logger(), "Active camera index exceeds max tilt speeds list size. Using default max tilt speed." );
    tilt_speed_ = 0.5;
  } else {
    tilt_speed_ = tilt_speeds_[active_camera];
  }

  std_msgs::msg::String msg;
  if ( active_camera_ >= camera_names_.size() ) {
    RCLCPP_WARN( node_->get_logger(), "Active camera index exceeds camera names list size. Cannot publish camera name." );
    msg.data = "Unknown Camera";
  } else {
    msg.data = camera_names_[active_camera];
  }

  camera_change_publisher_->publish( msg );
  RCLCPP_INFO( node_->get_logger(), "Switched to camera: %s", msg.data.c_str() );
  valid_subscription_ = true;
}

void VirtualCameraPlugin::update()
{
  if (!active_ || !valid_subscription_) {
    return;
  }

  if (pose_changed_) {
    // calculate new pose from current orientation
    current_orientation_ = Eigen::Affine3d(Eigen::AngleAxisd(current_pan_, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(current_tilt_, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()));
    current_orientation_ = base_transform_ * current_orientation_;

    // prepare for parameter
    std::vector<double> pose(6);
    pose[0] = current_orientation_.translation().x();
    pose[1] = current_orientation_.translation().y();
    pose[2] = current_orientation_.translation().z();
    Eigen::Vector3d euler_angles = current_orientation_.rotation().eulerAngles(2, 1, 0);
    pose[3] = euler_angles[2];
    pose[4] = euler_angles[1];
    pose[5] = euler_angles[0];
    param_client_->set_parameters({ rclcpp::Parameter("pose", pose) });
    pose_changed_ = false;

    RCLCPP_INFO( node_->get_logger(), "Updated camera pose: pan=%.2f, tilt=%.2f", pose[5], pose[4] );
  }
}

void VirtualCameraPlugin::activate()
{
  active_ = true;
}

void VirtualCameraPlugin::deactivate()
{
  active_ = false;
}

}  // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::VirtualCameraPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )
