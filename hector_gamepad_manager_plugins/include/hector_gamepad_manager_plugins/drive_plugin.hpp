#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP

#include <hector_gamepad_manager/gamepad_function_plugin.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

namespace hector_gamepad_manager_plugins
{
class DrivePlugin : public hector_gamepad_manager::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  void handleAxis( const std::string &function, double value ) override;

  void handleHold( const std::string &function ) override;

  void handleRelease( const std::string &function ) override;

  void update() override;

  void activate() override;

  void deactivate() override;

private:
  // Publisher for drive commands
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drive_command_publisher_;

  // Maximum linear speed
  double max_linear_speed_;

  // Maximum angular speed
  double max_angular_speed_;

  // Current drive value
  double drive_value_;

  // Current steer value
  double steer_value_;

  // Speed factor for slow driving
  double slow_factor_;

  // Speed factor for normal driving
  double normal_factor_;

  // Speed factor for fast driving
  double fast_factor_;

  // True if fast mode is active, false otherwise
  bool fast_mode_active_;

  // True if slow mode is active, false otherwise
  bool slow_mode_active_;

  // Current drive command
  geometry_msgs::msg::TwistStamped drive_command_;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP
