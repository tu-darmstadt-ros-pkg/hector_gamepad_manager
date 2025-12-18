#pragma once

#include <array>
#include <rclcpp/rclcpp.hpp>

#include <athena_firmware_interface_msgs/msg/battery_status.hpp>
#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>

namespace hector_gamepad_manager_plugins
{
class BatteryMonitorPlugin : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  void handlePress( const std::string &function, const std::string &id ) override;
  void handleRelease( const std::string &function, const std::string &id ) override { }
  void handleAxis( const std::string &function, const std::string &id, double value ) override { }

  void update() override { }

  void activate() override;
  void deactivate() override;

  double getVibrationFeedback() override;

private:
  void onBatteryStatus( const athena_firmware_interface_msgs::msg::BatteryStatus &msg );
  bool hasLowCell( const std::array<uint16_t, 8> &cells ) const;
  bool isMuted() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<athena_firmware_interface_msgs::msg::BatteryStatus>::SharedPtr battery_subscription_;

  hector::ParameterSubscription low_cell_threshold_param_;
  hector::ParameterSubscription vibration_intensity_param_;
  hector::ParameterSubscription mute_duration_param_;

  double low_cell_threshold_mv_{ 3300.0 };
  double vibration_intensity_{ 0.8 };
  double mute_duration_sec_{ 300.0 }; // default to 5 minutes

  bool low_voltage_detected_{ false };
  rclcpp::Time muted_until_{ 0, 0, RCL_ROS_TIME };
};
} // namespace hector_gamepad_manager_plugins
