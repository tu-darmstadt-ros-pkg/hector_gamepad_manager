#ifndef HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP
#define HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP

#include <hector_gamepad_manager/gamepad_function_plugin.hpp>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <yaml-cpp/yaml.h>

namespace hector_gamepad_manager
{
class HectorGamepadManager
{
public:
  HectorGamepadManager( const rclcpp::Node::SharedPtr &node );

private:
  struct ActionMapping {
    std::string plugin_name;
    std::string function_name;
  };

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  pluginlib::ClassLoader<GamepadFunctionPlugin> plugin_loader_;
  std::unordered_map<int, ActionMapping> button_mappings_;
  std::unordered_map<int, ActionMapping> axis_mappings_;
  std::unordered_map<std::string, std::shared_ptr<GamepadFunctionPlugin>> plugins_;

  bool loadConfig( const std::string &file_path );

  void initMappings( const YAML::Node &config, const std::string &type,
                     std::unordered_map<int, ActionMapping> &mappings );

  void loadPlugin( const std::string &plugin_name );

  void joyCallback( const sensor_msgs::msg::Joy::SharedPtr msg );
};
} // namespace hector_gamepad_manager

#endif // HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP
