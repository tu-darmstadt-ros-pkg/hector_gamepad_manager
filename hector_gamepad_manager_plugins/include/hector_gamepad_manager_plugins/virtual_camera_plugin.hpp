#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/transform.hpp>
#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace hector_gamepad_manager_plugins
{
// TODO try to parse in config as structs?
struct PanTiltBounds {
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

  void loadCameras();

  void poseCallback( const std::vector<rclcpp::Parameter> &parameters, const uint &camera_idx );

private:
  rclcpp::Node::SharedPtr node_;

  std::string back_camera_topic_name_ = "virtual_camera/back";
  std::string front_camera_topic_name_ = "virtual_camera/front";
  rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr front_transform_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr back_transform_publisher_;

  hector::ParameterSubscription pan_speed_param_sub_;
  hector::ParameterSubscription tilt_speed_param_sub_;
  hector::ParameterSubscription max_pan_param_sub_;
  hector::ParameterSubscription min_pan_param_sub_;
  hector::ParameterSubscription max_tilt_param_sub_;
  hector::ParameterSubscription min_tilt_param_sub_;
  hector::ParameterSubscription back_camera_topic_name_sub_;
  hector::ParameterSubscription front_camera_topic_name_sub_;
  hector::ParameterSubscription stick_hold_time_param_sub_;
  hector::ParameterSubscription invert_y_axis_param_sub_;

  double pan_speed_ = 0.5;
  double tilt_speed_ = 0.5;
  double max_pan_ = 3.14;
  double min_pan_ = -3.14;
  double max_tilt_ = 1.57;
  double min_tilt_ = -1.57;

  std::vector<double> back_transform_ = { 0.0, 0.0, 0.0 };
  std::vector<double> front_transform_ = { 0.0, 0.0, 0.0 };

  double stick_hold_time_ = 1.0;
  double stick_hold_timer_ = 0.0;
  double delta_t_ = 0.05; // Assuming fixed rate of 20Hz for update loop

  bool reverse_ = false;
  bool invert_y_axis_ = true;

  bool active_ = false;
  bool pose_changed_ = false;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP
