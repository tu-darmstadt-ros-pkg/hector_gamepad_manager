//
// Created by aljoscha-schmidt on 2/14/25.
//
#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_MANIPULATION_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_MANIPULATION_PLUGIN_HPP

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hector_gamepad_manager_interface/gamepad_plugin_interface.hpp>
#include <hector_gamepad_manager_plugins/controller_helper.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace hector_gamepad_manager_plugins
{
class ManipulationPlugin final : public hector_gamepad_manager_interface::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  std::string getPluginName() override;

  void handlePress( const std::string &function ) override;
  void handleRelease( const std::string &function ) override;

  void handleAxis( const std::string &function, const double value ) override;

  void update() override;

  void activate() override;

  void deactivate() override;

  /**
   * Avoid repeatedly sending zero cmd_vel commands.
   * @param linear_speed
   * @param angular_speed
   */
  void sendDriveCommand( double linear_speed, double angular_speed );

  bool isZeroCmd() const;

  /**
   * Reset all motion commands to zero.
   */
  void reset();

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr eef_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drive_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_cmd_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr hold_mode_client_;

  double max_eef_linear_speed_ = 0.0;
  double max_eef_angular_speed_ = 0.0;
  double max_gripper_speed_ = 0.0;
  double max_drive_linear_speed_ = 0.0;
  double max_drive_angular_speed_ = 0.0;

  bool hold_mode_active_ = false;
  bool hold_mode_change_requested_ = false;
  bool last_drive_cmd_zero_ = false;
  bool last_eef_cmd_zero_ = true;

  double move_left_right_ = 0.0;
  double move_up_down_ = 0.0;
  double move_forward_ = 0.0;
  double move_backward_ = 0.0;
  double rotate_pitch_ = 0.0;
  double rotate_yaw_ = 0.0;
  double rotate_roll_clockwise_ = 0.0;
  double rotate_roll_counter_clockwise_ = 0.0;
  double open_gripper_ = 0.0;
  double close_gripper_ = 0.0;

  std::string twist_controller_name_;
  std::vector<std::string> stop_controllers_;
  ControllerHelper controller_helper_{};
  geometry_msgs::msg::TwistStamped eef_cmd_;
  geometry_msgs::msg::TwistStamped drive_cmd_;
  std_msgs::msg::Float64 gripper_cmd_;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_MANIPULATION_PLUGIN_HPP
