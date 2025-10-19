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
 *  - "toggle/<var>"
 *  - "hold/<var>"
 *  - "set/<var>/to/<value>"
 *
 * Behavior:
 *  - toggle: invert the boolean value on press
 *  - hold:   set true on press, set false on release
 *  - set:    write <value> (string) on press
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
  void onToggle( const std::string &var, const std::string &topic, bool default_value );
  void onHoldPress( const std::string &var ) const;
  void onHoldRelease( const std::string &var ) const;
  void onSetString( const std::string &var, const std::string &value ) const;

  rclcpp::Node::SharedPtr node_;
  bool active_{ false };
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> toggle_publisher_;
};

} // namespace hector_gamepad_manager_plugins
