#ifndef HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP
#define HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP

#include "hector_gamepad_manager/gamepad_config.hpp"
#include "hector_gamepad_plugin_interface/feedback_manager.hpp"
#include "hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp"

#include <controller_orchestrator/controller_orchestrator.hpp>
#include <hector_gamepad_manager_msgs/msg/gamepad_mapping.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include <array>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace hector_gamepad_manager
{
class HectorGamepadManager
{
public:
  using GamepadFunctionPlugin = hector_gamepad_plugin_interface::GamepadFunctionPlugin;
  // Constructor
  explicit HectorGamepadManager( const rclcpp::Node::SharedPtr &node );

private:
  // What the manager owes the plugin for a double-press button. Buffering and Dispatched cannot
  // both hold - a press is either still held back to see whether a second one follows, or it has
  // gone out and the plugin is owed the release that ends it - which is what this buys over the
  // two booleans it replaced.
  enum class PressState {
    Idle,       // nothing pending
    Buffering,  // a first press is held back, waiting out the double-press window
    Dispatched, // a press was sent; a release is outstanding
  };

  // Per-button state for double-press detection
  struct ButtonTracker {
    bool pressed = false; // current physical state
    PressState state = PressState::Idle;
    rclcpp::Time last_press_time{ 0, 0, RCL_ROS_TIME };
  };

  // One entry of a config's "buttons"/"axis_buttons" section, resolved against the canonical
  // catalog: the name is the binding's identity, the id only indexes GamepadInputs::buttons.
  struct ButtonEntry {
    int id;
    std::string name;
    YAML::Node node;
  };

  // One Joy message translated into the canonical layout: axes in SDL order, buttons indexed by
  // the ids of gamepad_buttons.hpp, physical and axis-derived alike.
  struct GamepadInputs {
    std::array<float, kNumAxes> axes = {};
    std::array<bool, kNumButtons> buttons = {};
  };

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

  // Publish active configuration -> visualization in user interface
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_config_publisher_;

  // Publish the full static button/axis mapping of all loaded configs, once at startup.
  rclcpp::Publisher<hector_gamepad_manager_msgs::msg::GamepadMapping>::SharedPtr mapping_publisher_;

  // Class loader for the gamepad function plugins
  pluginlib::ClassLoader<GamepadFunctionPlugin> plugin_loader_;

  std::map<std::string, GamepadConfig> configs_;

  // Maps buttons to the config they switch to
  std::array<ConfigSwitch, kNumButtons> config_switch_button_mapping_;

  // Name of the active configuration
  std::string active_config_;

  // Name of the config activated at startup
  std::string default_config_;

  // Config directory relative to package share or absolute path override
  std::string config_directory_;

  // Map of loaded plugins
  std::unordered_map<std::string, std::shared_ptr<GamepadFunctionPlugin>> plugins_;

  // stores the plugins present in the active configuration file
  std::vector<std::shared_ptr<GamepadFunctionPlugin>> active_plugins_;

  // Blackboard for inter-plugin communication and function arguments
  std::shared_ptr<hector_gamepad_plugin_interface::Blackboard> blackboard_;

  // Feedback manager for handling vibration patterns
  std::shared_ptr<hector_gamepad_plugin_interface::FeedbackManager> feedback_manager_;

  // Controller Orchestrator for activating controllers
  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> controller_orchestrator_;

  // Per-button trackers for double-press detection, indexed by button id and reset on every
  // config switch. Small enough to hold one entry per known button outright.
  std::array<ButtonTracker, kNumButtons> button_trackers_;

  // Double-press window in seconds (ROS param `double_press_window_sec`, default 0.25).
  double double_press_window_sec_;

  // Whether the last joy message came from a source with the expected layout. Only used to act
  // once on the transition to a rejected source, not to remember a verdict: every message is
  // checked on its own.
  bool joy_source_ok_ = true;

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
   * @brief Initialize the button mappings from the "buttons" and "axis_buttons" sections.
   *
   * @param config The YAML node containing the configuration.
   * @param mappings The mappings to be initialized.
   * @return True if the mappings were initialized successfully, false otherwise.
   */
  bool initButtonMappings( const YAML::Node &config, const std::string &config_name,
                           std::map<int, ButtonFunctionMapping> &mappings );

  /**
   * @brief Resolve the "buttons" and "axis_buttons" sections of a config. Both are keyed by
   * canonical name and differ only in whether the button is one the gamepad reports.
   *
   * @return False if the "buttons" section is missing, or a key is not a known button name, or it
   * is written in the wrong one of the two sections.
   */
  bool collectButtonEntries( const YAML::Node &config, std::vector<ButtonEntry> &entries );

  // Initialize the axis mappings from the "axes" section.
  bool initAxisMappings( const YAML::Node &config, const std::string &config_name,
                         std::map<int, FunctionMapping> &mappings );

  // The named plugin, loaded into plugins_ on first use. Null if it could not be loaded.
  std::shared_ptr<GamepadFunctionPlugin> loadPlugin( const std::string &plugin_name );

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
   * @brief Emit the press a Buffering tracker was holding back, because it turned out to be a
   * single press rather than the first half of a double one.
   *
   * @param still_held True if the button is down *from that press*, so the coming messages will
   * produce its hold and release through the normal path. False pairs the press with an immediate
   * release, which is what a quick tap needs and what a press superseded by a new one needs - the
   * button being down again does not make the old press still held.
   */
  void dispatchBufferedPress( const ButtonFunctionMapping &mapping, ButtonTracker &tracker,
                              bool still_held );

  // Synthesize the events needed to bring plugins back to a "no button held" state before a
  // config switch. Not wired to shutdown: the manager has no destructor hook, so a process going
  // down leaves the last press unresolved.
  void flushPendingButtonState();

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
  GamepadInputs convertJoyToGamepadInputs( const sensor_msgs::msg::Joy &msg );

  /**
   * @brief Check a Joy message against the layout game_controller_node publishes and log how to
   * fix the launch if it does not match. Runs on every message, so a source that is relaunched or
   * joined by a second publisher mid-session is caught too. Reporting is throttled.
   *
   * @return True if the message may be dispatched. A false means the ids in it address different
   * controls than the ones the configs are written against, so acting on it would command
   * whatever happens to sit at the same index - the message is dropped instead.
   */
  bool checkJoySource( const sensor_msgs::msg::Joy &msg );

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
