#ifndef HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP
#define HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP

#include "hector_gamepad_manager/gamepad_function_plugin.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <yaml-cpp/yaml.h>

namespace hector_gamepad_manager
{
class HectorGamepadManager
{
public:
  // Constructor
  explicit HectorGamepadManager( const rclcpp::Node::SharedPtr &node );

private:
  // Struct to store the mapping of a button or axis to a function of a plugin
  struct FunctionMapping {
    // Name of the plugin
    std::string plugin_name;

    // Name of the function
    std::string function_name;
  };

  // Struct to store the inputs from the gamepad
  struct GamepadInputs {
    // Vector of axes values
    std::vector<float> axes = std::vector( 8, 0.0f );

    // Vector of button states
    std::vector<bool> buttons = std::vector( 25, false );
  };

  // ROS node
  rclcpp::Node::SharedPtr node_;

  // Subscription to the joy topic
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

  // Current gamepad inputs
  GamepadInputs inputs_;

  // Class loader for the gamepad function plugins
  pluginlib::ClassLoader<GamepadFunctionPlugin> plugin_loader_;

  // Map of button mappings for each configuration
  std::map<std::string, std::unordered_map<int, FunctionMapping>> button_mappings_;

  // Map of axis mappings for each configuration
  std::map<std::string, std::unordered_map<int, FunctionMapping>> axis_mappings_;

  // Name of the active configuration
  std::string active_config_;

  // Flag to indicate if the first configuration has been loaded
  bool first_config_;

  // Pointer to the active button mappings
  std::unordered_map<int, FunctionMapping> *active_button_mappings_;

  // Pointer to the active axis mappings
  std::unordered_map<int, FunctionMapping> *active_axis_mappings_;

  // Map of loaded plugins
  std::unordered_map<std::string, std::shared_ptr<GamepadFunctionPlugin>> plugins_;

  // Deadzone to consider an axis as pressed
  static constexpr float AXIS_DEADZONE = 0.5;

  /**
   * @brief Load the configuration file.
   *
   * @param file_path The path to the configuration file.
   * @return True if the configuration file was loaded successfully, false otherwise.
   */
  bool loadConfig( const std::string &file_path );

  /**
   * @brief Initialize the mappings for buttons or axes.
   *
   * @param config The YAML node containing the configuration.
   * @param type The type of the mapping (buttons or axes).
   * @param mappings The mappings to be initialized.
   * @return True if the mappings were initialized successfully, false otherwise.
   */
  bool initMappings( const YAML::Node &config, const std::string &type,
                     std::unordered_map<int, FunctionMapping> &mappings );

  /**
   * @brief Callback function for the joy topic.
   *
   * @param msg The message containing the gamepad inputs.
   */
  void joyCallback( sensor_msgs::msg::Joy::SharedPtr msg );

  /**
   * @brief Convert the joy message to gamepad inputs.
   *
   * @param msg The message containing the gamepad inputs.
   */
  void convert_joy_to_gamepad_inputs( const sensor_msgs::msg::Joy::SharedPtr &msg );
};
} // namespace hector_gamepad_manager

#endif // HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP
