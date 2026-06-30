//
// Created by aljoscha-schmidt on 2/14/25.
//
#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_MANIPULATION_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_MANIPULATION_PLUGIN_HPP

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace hector_gamepad_manager_plugins
{
class ManipulationPlugin final : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  void handlePress( const std::string &function, const std::string &id ) override;
  void handleRelease( const std::string &function, const std::string &id ) override;

  void handleAxis( const std::string &function, const std::string &id, const double value ) override;

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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr nullspace_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr hold_mode_client_;

  hector::ParameterSubscription max_eef_linear_speed_param_sub_;
  hector::ParameterSubscription max_eef_angular_speed_param_sub_;
  hector::ParameterSubscription max_gripper_speed_param_sub_;
  hector::ParameterSubscription max_drive_linear_speed_param_sub_;
  hector::ParameterSubscription max_drive_angular_speed_param_sub_;
  hector::ParameterSubscription eef_twist_frame_param_sub_;

  hector::ParameterSubscription max_nullspace_joint_speed_param_sub_;
  hector::ParameterSubscription max_joint_speed_param_sub_;

  double max_eef_linear_speed_ = 0.0;
  double max_eef_angular_speed_ = 0.0;
  double max_gripper_speed_ = 0.0;
  double max_drive_linear_speed_ = 0.0;
  double max_drive_angular_speed_ = 0.0;
  double max_nullspace_joint_speed_ = 0.0;
  double max_joint_speed_ = 0.0;

  bool hold_mode_active_ = false;
  // While nullspace_mode is held, the left joystick biases arm joints (keeping the eef pose)
  // instead of commanding eef/base motion.
  bool nullspace_mode_active_ = false;
  bool last_nullspace_cmd_zero_ = true;
  // While single_joint_mode is held, both joysticks drive the first 4 arm joints directly
  // (left X->j1, left Y->j2, right X->j3, right Y->j4); the eef pose is not held.
  bool single_joint_mode_active_ = false;
  bool last_joint_cmd_zero_ = true;
  // Number of active arm joints (size of the nullspace command vector).
  int num_arm_joints_ = 0;
  bool hold_mode_change_requested_ = false;
  bool last_drive_cmd_zero_ = false;
  bool last_eef_cmd_zero_ = true;
  bool enable_drive_cmd_ = true;

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
  std::string gripper_controller_name_;
  geometry_msgs::msg::TwistStamped eef_cmd_;
  geometry_msgs::msg::TwistStamped drive_cmd_;
  std_msgs::msg::Float64 gripper_cmd_;

  /// Publishes a zeroed nullspace command (used to stop biasing).
  void publishZeroNullspaceCmd();

  /// Publishes a zeroed direct-joint command (used to stop single-joint jogging).
  void publishZeroJointCmd();
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_MANIPULATION_PLUGIN_HPP
