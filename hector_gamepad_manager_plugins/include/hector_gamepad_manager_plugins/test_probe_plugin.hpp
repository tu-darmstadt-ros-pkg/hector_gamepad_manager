#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_TEST_PROBE_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_TEST_PROBE_PLUGIN_HPP

#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>

namespace hector_gamepad_manager_plugins
{
class TestProbePlugin final : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  void handleAxis( const std::string &function, const std::string &id, double value ) override;
  void handlePress( const std::string &function, const std::string &id ) override;
  void handleHold( const std::string &function, const std::string &id ) override;
  void handleRelease( const std::string &function, const std::string &id ) override;

  void update() override { }
  void activate() override { active_ = true; }
  void deactivate() override { active_ = false; }

private:
  void publishEvent( const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub,
                     const std::string &event, const std::string &function, const std::string &id,
                     const std::string &value = "" );

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr press_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hold_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr release_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr axis_pub_;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_TEST_PROBE_PLUGIN_HPP
