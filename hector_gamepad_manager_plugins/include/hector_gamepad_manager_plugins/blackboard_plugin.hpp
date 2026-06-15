#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>
#include <std_msgs/msg/bool.hpp>
namespace hector_gamepad_manager_plugins
{

/**
 * BlackboardPlugin
 *
 * Button mappings support the following function strings:
 *  - "toggle" with args: name, [topic], [ocs_topic], [initial]
 *  - "hold" with args: name
 *  - "set" with args: name, set
 *
 * Behavior:
 *  - toggle: invert the boolean value on press
 *  - hold:   set true on press, set false on release
 *  - set:    write <value> (string) on press
 *
 * Topic options (both are relative to the robot namespace the node runs in):
 *  - topic: publishes the new value to this topic
 *  - ocs_topic: optional second topic the new value is also published to (e.g. for UI feedback);
 *               ignored if empty or equal to topic
 */
class BlackboardPlugin final : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  // Button interface
  void handlePress( const std::string &function, const std::string &id ) override;
  void handleRelease( const std::string &function, const std::string &id ) override;

  // Axis interface (unused here)
  void handleAxis( const std::string &function, const std::string &id, const double value ) override
  {
  }

  // Lifecycle-ish
  void activate() override;
  void deactivate() override;
  void update() override { }

private:
  void onToggle( const std::string &var, const std::string &topic, const std::string &ocs_topic,
                 bool initial_value );
  void onHoldPress( const std::string &var ) const;
  void onHoldRelease( const std::string &var ) const;
  void onSetString( const std::string &var, const std::string &value ) const;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr getOrCreatePublisher( const std::string &topic );

  bool active_{ false };
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> publishers_;
};

} // namespace hector_gamepad_manager_plugins
