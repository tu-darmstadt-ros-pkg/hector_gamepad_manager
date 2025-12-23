#pragma once

#include <rclcpp/rclcpp.hpp>

#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>
#include <hector_gamepad_plugin_interface/vibration_pattern.hpp>
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include <vector>

namespace hector_gamepad_manager_plugins
{

class BatteryMonitorPlugin : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  void handlePress( const std::string &function, const std::string &id ) override;
  void handleRelease( const std::string &function, const std::string &id ) override { }
  void handleAxis( const std::string &function, const std::string &id, double value ) override { }

  void activate() override;
  void deactivate() override;
  void update() override;

private:
  void trySubscribe();
  void onBatteryMessage( const ros_babel_fish::CompoundMessage::SharedPtr &msg );
  bool isMuted() const;

  rclcpp::Node::SharedPtr node_;
  ros_babel_fish::BabelFish::SharedPtr fish_;
  ros_babel_fish::BabelFishSubscription::SharedPtr battery_subscription_;
  rclcpp::TimerBase::SharedPtr subscription_timer_;

  hector::ParameterSubscription low_cell_threshold_param_;
  hector::ParameterSubscription mute_duration_param_;
  hector::ParameterSubscription ignore_zero_voltage_param_;
  hector::ParameterSubscription ignore_nan_voltage_param_;

  double low_cell_threshold_{ 3300.0 };
  double mute_duration_sec_{ 300.0 }; // default to 5 minutes
  bool ignore_zero_voltage_{ true };
  bool ignore_nan_voltage_{ true };

  std::vector<std::string> cell_voltage_fields_;

  bool low_voltage_detected_{ false };
  rclcpp::Time muted_until_{ 0, 0, RCL_ROS_TIME };
  std::string vibration_pattern_id_;
  std::shared_ptr<hector_gamepad_plugin_interface::VibrationPattern> vibration_pattern_;
};
} // namespace hector_gamepad_manager_plugins
