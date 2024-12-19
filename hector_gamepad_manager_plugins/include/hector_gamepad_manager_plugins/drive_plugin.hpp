#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP

#include <hector_gamepad_manager/gamepad_function_plugin.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

namespace hector_gamepad_manager_plugins
{
class DrivePlugin : public hector_gamepad_manager::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node, const bool active ) override;

  void handleButton( const std::string &function, const bool pressed ) override;

  void handleAxis( const std::string &function, const double value ) override;

  void update() override;

  void activate() override;

  void deactivate() override;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drive_command_publisher_;

  double max_linear_speed_;
  double max_angular_speed_;

  double drive_value_;
  double steer_value_;

  double slow_factor_;
  double normal_factor_;
  double fast_factor_;

  bool fast_button_pressed_;
  bool slow_button_pressed_;

  std::string drive_command_topic_;
  geometry_msgs::msg::TwistStamped drive_command_;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP
