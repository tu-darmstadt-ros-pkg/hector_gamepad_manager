//
// Created by aljoscha-schmidt on 2/14/25.
//
#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_MANIPULATION_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_MANIPULATION_PLUGIN_HPP

#include <hector_gamepad_manager/gamepad_function_plugin.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
namespace hector_gamepad_manager_plugins
{
class ManipulationPlugin : public hector_gamepad_manager::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node, const bool active ) override;

  void handleButton( const std::string &function, const bool pressed ) override;

  void handleAxis( const std::string &function, const double value ) override;

  void update() override;

  void activate() override;

  void deactivate() override;

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

  double move_left_right_ = 0.0;
  double move_up_down_ = 0.0;
  double move_forward_ = 0.0;
  double move_backward_ = 0.0;
  double rotate_pitch = 0.0;
  double rotate_yaw = 0.0;
  double rotate_roll_clockwise = 0.0;
  double rotate_roll_counter_clockwise = 0.0;
  double open_gripper = 0.0;
  double close_gripper = 0.0;

  geometry_msgs::msg::TwistStamped eef_cmd_;
  geometry_msgs::msg::TwistStamped drive_cmd_;
  std_msgs::msg::Float64 gripper_cmd_;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_MANIPULATION_PLUGIN_HPP
