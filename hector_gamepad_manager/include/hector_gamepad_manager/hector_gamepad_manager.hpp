#ifndef HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP
#define HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP

#include "hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

namespace hector_gamepad_manager
{
class HectorGamepadManager
{
public:
  using GamepadFunctionPlugin = hector_gamepad_plugin_interface::GamepadFunctionPlugin;
  // Constructor
  explicit HectorGamepadManager( const rclcpp::Node::SharedPtr &node );

private:
  // Struct to store the mapping of a button or axis to a function of a plugin
  struct FunctionMapping {
    // Name of the plugin
    std::shared_ptr<GamepadFunctionPlugin> plugin;

    // Name of the function
    std::string function_name;
  };

  // Struct to store the inputs from the gamepad
  struct GamepadInputs {
    // Vector of axes values
    std::array<float, 8> axes = std::array<float, 8>{ 0.0 };

    std::array<bool, 25> buttons = std::array<bool, 25>{ false };
  };

  struct GamepadConfig {
    std::unordered_map<int, FunctionMapping> button_mappings;
    std::unordered_map<int, FunctionMapping> axis_mappings;
  };

  rclcpp::Node::SharedPtr robot_ns_node_;
  rclcpp::Node::SharedPtr ocs_ns_node_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr joy_feedback_publisher_;

  // Publish active configuration -> visualization in user interface
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_config_publisher_;

  // Class loader for the gamepad function plugins
  pluginlib::ClassLoader<GamepadFunctionPlugin> plugin_loader_;

  std::map<std::string, GamepadConfig> configs_;

  // Maps buttons to config names
  std::array<std::string, 25> config_switch_button_mapping_;

  // Name of the active configuration
  std::string active_config_;

  // Name of the config activated at startup
  std::string default_config_;

  // used for to prefix the configs
  std::string robot_namespace_;

  // namespace for the operator station
  std::string ocs_namespace_;

  // Map of loaded plugins
  std::unordered_map<std::string, std::shared_ptr<GamepadFunctionPlugin>> plugins_;

  // stores the plugins present in the active configuration file
  std::vector<std::shared_ptr<GamepadFunctionPlugin>> active_plugins_;

  std::shared_ptr<hector_gamepad_plugin_interface::Blackboard> blackboard_;

  // Deadzone to consider an axis as pressed
  static constexpr float AXIS_DEADZONE = 0.5;

  /**
   * @brief Load the Config Switch File, determines which configs must be loaded
   * @param file_name
   * @return
   */
  bool loadConfigSwitchesConfig( const std::string &file_name );

  /**
   * @brief Load the configuration file.
   *
   * @param file_name The name of the configuration file.
   * @return True if the configuration file was loaded successfully, false otherwise.
   */
  bool loadConfig( const std::string &pkg_name, const std::string &file_name );

  /**
   * @brief Handle the configuration switches.
   *
   * @return True if the configuration switching is in progress and normal button / axis behavior should be ignored
   */
  bool handleConfigurationSwitches( const GamepadInputs &inputs );

  /**
   * @brief Switch the active configuration.
   *
   * @param config_name The name of the configuration to switch to.
   * @return True if switching was successful, false otherwise.
   */
  bool switchConfig( const std::string &config_name );

  /**
   * @brief Initialize the mappings for buttons or axes.
   *
   * @param config The YAML node containing the configuration.
   * @param type The type of the mapping (buttons or axes).
   * @param mappings The mappings to be initialized.
   * @return True if the mappings were initialized successfully, false otherwise.
   */
  bool initMappings( const YAML::Node &config, const std::string &type,
                     const std::string &config_name,
                     std::unordered_map<int, FunctionMapping> &mappings );

  /**
   * @brief Activates all plugins present in the given config
   * @param config_name
   */
  void activatePlugins( const std::string &config_name );

  /**
   * @brief Deactivates all plugins
   */
  void deactivatePlugins();

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
   * @return the transformed gamepad inputs
   */
  GamepadInputs convertJoyToGamepadInputs( const sensor_msgs::msg::Joy::SharedPtr &msg );

  /**
   * @brief Get the path of a file in a package. Assuming the file is in the config folder.
   *
   * @param pkg_name The name of the package.
   * @param file_name The name of the file.
   * @return The path of the file.
   */
  std::string getPath( const std::string &pkg_name, const std::string &file_name );
};
} // namespace hector_gamepad_manager

#endif // HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP
