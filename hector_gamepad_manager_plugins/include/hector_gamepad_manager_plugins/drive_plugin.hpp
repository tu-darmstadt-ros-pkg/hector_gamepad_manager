#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP

#include <hector_gamepad_manager_interface/gamepad_plugin_interface.hpp>


#include <geometry_msgs/msg/twist_stamped.hpp>

namespace hector_gamepad_manager_plugins
{
class DrivePlugin : public hector_gamepad_manager_interface::GamepadFunctionPlugin
{
public:

  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  std::string getPluginName() override;

  void handleAxis( const std::string &function, double value ) override;

  void handlePress( const std::string &function ) override;

  void handleRelease( const std::string &function ) override;

  void update() override;

  void activate() override;

  void deactivate() override;

private:
  void sendDriveCommand( double linear_speed, double angular_speed );

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drive_command_publisher_;
  geometry_msgs::msg::TwistStamped drive_command_;

  double max_linear_speed_ = 0.0;
  double max_angular_speed_ = 0.0;

  double drive_value_ = 0.0;
  double steer_value_ = 0.0;

  double slow_factor_ = 0.0;
  double normal_factor_ = 0.0;
  double fast_factor_ = 0.0;
  bool fast_mode_active_ = false;
  bool slow_mode_active_ = false;
  bool last_cmd_zero_ = false;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP
