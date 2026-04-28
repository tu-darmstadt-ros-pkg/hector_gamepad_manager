#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_FLIPPER_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_FLIPPER_PLUGIN_HPP

#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>

#include <hector_ros_controllers_msgs/action/drive_flipper_group.hpp>
#include <hector_ros_controllers_msgs/action/sync_flipper_group.hpp>

namespace hector_gamepad_manager_plugins
{
class FlipperPlugin : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  void handlePress( const std::string &function, const std::string &id ) override;

  void handleRelease( const std::string &function, const std::string &id ) override;

  void handleAxis( const std::string &function, const std::string &id, const double value ) override;

  void update() override;

  void activate() override;

  void deactivate() override;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr flipper_command_publisher_;

  hector::ParameterSubscription speed_sub_;
  hector::ParameterSubscription flipper_front_factor_param_sub_;
  hector::ParameterSubscription flipper_back_factor_param_sub_;
  hector::ParameterSubscription button_speed_scaling_param_sub_;

  double speed_ = 0.0;
  double flipper_front_factor_ = 0.0;
  double flipper_back_factor_ = 0.0;
  double button_speed_scaling_ = 0.75;

  bool individual_front_flipper_mode_ = false;
  bool individual_back_flipper_mode_ = false;

  std::string standard_controller_;
  std::vector<std::string> teleop_controller_;

  std_msgs::msg::Float64MultiArray flipper_speed_commands_;

  std::array<double, 4> button_vel_commands_ = {};
  std::array<double, 4> axis_vel_commands_ = {};
  std::array<double, 4> vel_commands_ = {};

  bool last_cmd_zero_ = false;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handler_;

  void handleBasicControlInput( double base_speed_factor, bool is_button,
                                const std::string &function );
  void handleIndividualFlipperControlInput( const double base_speed_factor, const bool is_button,
                                            const std::string &function );

  void handleFlipperUpright( const std::string &function );
  void handleFlipperSync( const std::string &function );

  void resetCommands();

  void publishCommands() const;

  bool checkCurrentCmdIsZero() const;

  // Action clients for flipper group actions
  using DriveFlipperGroupAction = hector_ros_controllers_msgs::action::DriveFlipperGroup;
  using SyncFlipperGroupAction = hector_ros_controllers_msgs::action::SyncFlipperGroup;

  rclcpp_action::Client<DriveFlipperGroupAction>::SharedPtr drive_flipper_client_;
  rclcpp_action::Client<SyncFlipperGroupAction>::SharedPtr sync_flipper_client_;

  // Non-null while a goal is accepted and pending; duplicate requests are dropped.
  rclcpp_action::ClientGoalHandle<DriveFlipperGroupAction>::SharedPtr drive_in_flight_;
  rclcpp_action::ClientGoalHandle<SyncFlipperGroupAction>::SharedPtr sync_in_flight_;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_FLIPPER_PLUGIN_HPP
