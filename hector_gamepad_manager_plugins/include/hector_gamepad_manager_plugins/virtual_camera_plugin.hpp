#ifndef  HECTOR_GAMEPAD_MANAGER_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP

#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>
#include <eigen3/Eigen/Dense>

namespace hector_gamepad_manager_plugins
{
  // TODO try to parse in config as structs?
struct PanTiltBounds
{
  double min_pan;
  double max_pan;
  double min_tilt;
  double max_tilt;
  double base_pan;
  double base_tilt;
};

class VirtualCameraPlugin : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  void handleAxis( const std::string &function, const std::string &id, const double value ) override;

  void handlePress( const std::string &function, const std::string &id ) override;

  void handleHold( const std::string &function, const std::string &id ) override;

  void handleRelease( const std::string &function, const std::string &id ) override;

  void update() override;

  void activate() override;

  void deactivate() override;

  void loadCamera( const uint &camera_idx );

  void poseCallback( const std::vector<rclcpp::Parameter> &parameters, const uint &camera_idx );
private:
  std::vector<std::string> node_names_ = {"camera_1"};
  std::vector<std::string> camera_names_ = {"Camera 1"};
  rclcpp::AsyncParametersClient::SharedPtr param_client_;

  // TODO try to parse as struct directly from yaml?
  std::vector<double> base_tilts_ = {0.0};
  std::vector<double> base_pans_ = {0.0};
  std::vector<double> base_rolls_ = {0.0};
  std::vector<double> max_pans_ = {0.0};
  std::vector<double> min_pans_ = {0.0};
  std::vector<double> max_tilts_ = {0.0};
  std::vector<double> min_tilts_ = {0.0};
  std::vector<double> pan_speeds_ = {0.0};
  std::vector<double> tilt_speeds_ = {0.0};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_change_publisher_;

  hector::ParameterSubscription pan_speed_param_sub_;
  hector::ParameterSubscription tilt_speed_param_sub_;
  hector::ParameterSubscription max_pan_param_sub_;
  hector::ParameterSubscription min_pan_param_sub_;
  hector::ParameterSubscription max_tilt_param_sub_;
  hector::ParameterSubscription min_tilt_param_sub_;
  hector::ParameterSubscription node_names_param_sub_;
  hector::ParameterSubscription camera_names_param_sub_;
  hector::ParameterSubscription stick_hold_time_param_sub_;
  hector::ParameterSubscription base_tilts_param_sub_;
  hector::ParameterSubscription base_pans_param_sub_;
  hector::ParameterSubscription base_rolls_param_sub_;

  // TODO
  //std::vector<PanTiltBounds> camera_bounds_;

  std::vector<double> base_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // x, y, z, roll, pitch, yaw

  double pan_speed_ = 0.5;
  double tilt_speed_ = 0.5;
  double max_pan_ = 3.14159;
  double min_pan_ = -3.14159;
  double max_tilt_ = 1.5708;
  double min_tilt_ = -1.5708;
  
  Eigen::Affine3d base_transform_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d current_orientation_ = Eigen::Affine3d::Identity();

  double current_pan_ = 0.0;
  double current_tilt_ = 0.0;

  double stick_hold_time_ = 1.0;

  uint active_camera_ = 0;

  double delta_t_ = 0.05; // Assuming fixed rate of 20Hz for update loop

  double stick_hold_timer_ = 0.0;

  bool active_ = false;
  bool pose_changed_ = false;
  bool valid_subscription_ = false;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP