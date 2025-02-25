#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_FLIPPER_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_FLIPPER_PLUGIN_HPP

#include <hector_gamepad_manager/gamepad_function_plugin.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "hector_gamepad_manager_plugins/controller_helper.hpp"

namespace hector_gamepad_manager_plugins
{
class FlipperPlugin : public hector_gamepad_manager::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node) override;

  std::string getPluginName() override;

  void handlePress( const std::string &function ) override;

  void handleHold( const std::string &function ) override;

  void handleAxis( const std::string &function, const double value ) override;

  void update() override;

  void activate() override;

  void deactivate() override;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr flipper_command_publisher_;

  double speed_;
  double flipper_front_factor_;
  double flipper_back_factor_;

  std::string standard_controller_;
  std::string teleop_controller_;

  ControllerHelper controller_helper_;

  std_msgs::msg::Float64MultiArray flipper_speed_commands_;

  std::vector<double> vel_commands_;
  bool last_cmd_zero_;

  void set_front_flipper_command(double vel);

  void set_back_flipper_command(double vel);

  void reset_commands();

  void publish_commands();

  bool check_current_cmd_is_zero();

};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_DRIVE_PLUGIN_HPP
