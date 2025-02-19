#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP

#include <hector_gamepad_manager/gamepad_function_plugin.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

namespace hector_gamepad_manager_plugins
{
class DrivePlugin : public hector_gamepad_manager::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node, const std::string &robot_name ) override;

  void handleAxis( const std::string &function, double value ) override;

  void handlePress( const std::string &function ) override;

  void handleRelease( const std::string &function ) override;

  void switchControlledRobot( const std::string &robot_name ) override;

  void update() override;

  void activate() override;

  void deactivate() override;

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drive_command_publisher_;
  geometry_msgs::msg::TwistStamped drive_command_;

  double max_linear_speed_;
  double max_angular_speed_;
  double drive_value_;
  double steer_value_;
  double slow_factor_;
  double normal_factor_;
  double fast_factor_;
  bool fast_mode_active_;
  bool slow_mode_active_;
  std::string cmd_vel_topic;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP
