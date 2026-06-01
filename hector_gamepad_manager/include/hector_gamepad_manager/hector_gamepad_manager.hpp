#ifndef HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP
#define HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP

#include "hector_gamepad_plugin_interface/feedback_manager.hpp"
#include "hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp"

#include <controller_orchestrator/controller_orchestrator.hpp>
#include <hector_gamepad_manager_msgs/srv/switch_config.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include <map>
#include <memory>
#include <unordered_map>

namespace hector_gamepad_manager
{
class HectorGamepadManager
{
public:
  using GamepadFunctionPlugin = hector_gamepad_plugin_interface::GamepadFunctionPlugin;
  using SwitchConfig = hector_gamepad_manager_msgs::srv::SwitchConfig;

  // Parameter defaults, shared with the node executable so both sites agree.
  static constexpr const char *DEFAULT_CONFIG_NAME = "athena";
  static constexpr const char *DEFAULT_CONFIG_DIRECTORY = "config";
  static constexpr const char *DEFAULT_PLUGIN_PARAMS = "athena";

  // Constructor
  explicit HectorGamepadManager( const rclcpp::Node::SharedPtr &node );

  /**
   * @brief Block on the manager-owned executor (spins the OCS node and every robot node).
   *
   * The manager owns a SingleThreadedExecutor; robot nodes are added to it as they are built.
   * Called by the node executable instead of rclcpp::spin(). Never spun by tests.
   */
  void spin();

  /**
   * @brief Apply a named plugin-param set to the ACTIVE robot's plugins at runtime.
   *
   * Thin wrapper around the per-robot overload operating on the active robot. See that overload
   * for the snapshot/restore/reset semantics. Note: the SwitchConfig service does NOT use this for
   * switching param sets (it rebuilds the robot node so plugins re-declare with fresh overrides,
   * because not all plugins read their parameters at runtime); this remains a runtime-tuning
   * primitive for reconfigurable parameters.
   */
  void applyPluginParamSet( const std::string &name, bool reset );

  /// The currently controlled robot's node (its FQN carries the robot namespace). For tests.
  rclcpp::Node::SharedPtr robotNode() const { return active_ ? active_->node : nullptr; }

private:
  // Fixed gamepad input dimensions. Buttons 0-10 are physical, 11-24 are virtual (axis-as-button).
  // Config-YAML ids are validated against these bounds before they index the fixed input arrays.
  static constexpr int NUM_BUTTONS = 25;
  static constexpr int NUM_AXES = 8;

  // Struct to store the mapping of an axis to a function of a plugin
  struct FunctionMapping {
    // Name of the plugin
    std::shared_ptr<GamepadFunctionPlugin> plugin;

    // Name of the function
    std::string function_name;
  };

  // For double-press buttons the manager bypasses handleButton and dispatches handlePress / handleHold / handleRelease directly, so plugins must not rely on button_states_ for them.
  struct ButtonFunctionMapping {
    std::shared_ptr<GamepadFunctionPlugin> plugin;

    std::string on_press;        // function called on initial press
    std::string on_double_press; // function called on double press (empty = disabled)
    std::string on_hold;         // function called while held (empty = uses on_press)
    std::string on_release;      // function called on release (empty = uses on_press)

    bool has_double_press() const { return !on_double_press.empty(); }
  };

  // Per-button state for double-press detection
  struct ButtonTracker {
    bool pressed = false;          // current physical state
    bool press_dispatched = false; // was on_press already sent?
    bool awaiting_double_press = false;
    rclcpp::Time last_press_time{ 0, 0, RCL_ROS_TIME };
  };

  // Struct to store the inputs from the gamepad
  struct GamepadInputs {
    // Vector of axes values
    std::array<float, NUM_AXES> axes = std::array<float, NUM_AXES>{ 0.0 };

    std::array<bool, NUM_BUTTONS> buttons = std::array<bool, NUM_BUTTONS>{ false };
  };

  struct GamepadConfig {
    std::unordered_map<int, ButtonFunctionMapping> button_mappings;
    std::unordered_map<int, FunctionMapping> axis_mappings;
  };

  /**
   * @brief All state belonging to one controlled robot namespace.
   *
   * Cached and keyed by robot namespace in `robots_`. Switching away deactivates the plugins (they
   * stay alive but inert) while the node, publishers/subscribers and blackboard survive, so
   * switching back just re-activates. Each robot runs on its OWN real node carrying the robot
   * namespace and the plugin-param overrides: a sub-node could not, because it would share the
   * base node's parameter interface and a second robot re-declaring the same plugin param collides.
   */
  struct RobotControl {
    std::string robot_namespace;        // e.g. "athena"
    std::string config_switches_name;   // the config-switches FILE (service field config_name)
    std::string plugin_params_name;     // applied plugin-param set

    rclcpp::Node::SharedPtr node;       // fresh real node, ns=/<robot_namespace>,
                                        //   name=<ocs_namespace>_gamepad_robot_control
    std::shared_ptr<controller_orchestrator::ControllerOrchestrator> controller_orchestrator;
    std::shared_ptr<hector_gamepad_plugin_interface::Blackboard> blackboard;

    std::unordered_map<std::string, std::shared_ptr<GamepadFunctionPlugin>> plugins;
    std::vector<std::shared_ptr<GamepadFunctionPlugin>> active_plugins;
    std::map<std::string, GamepadConfig> configs;
    std::array<std::string, NUM_BUTTONS> config_switch_button_mapping;
    std::string active_config;   // last within-robot button/axis config (restored on switch-back)
    std::string default_config;  // within-robot default config (from the switches file)
    std::unordered_map<int, ButtonTracker> button_trackers;

    // plugin-param snapshot/restore state, scoped to this robot's node
    std::vector<std::string> active_plugin_param_names;
    std::unordered_map<std::string, std::vector<rclcpp::Parameter>> plugin_params_cache;
  };

  // ---- OCS-shared state (created once, survives switches) -------------------------------------

  // Base node passed to the constructor; kept for the executor and use_sim_time propagation.
  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr ocs_ns_node_;

  // Owns all nodes (OCS + every robot node, active or inactive). Robot subscriptions/action
  // clients only make progress while this spins.
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

  // Publish active button/axis configuration -> visualization in user interface
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_config_publisher_;

  // Latched publish of the active robot namespace -> lets the UI highlight the controlled robot.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_robot_publisher_;

  // Service to retarget the gamepad to another robot namespace / config / plugin-param set.
  rclcpp::Service<SwitchConfig>::SharedPtr switch_config_service_;

  // Class loader for the gamepad function plugins
  pluginlib::ClassLoader<GamepadFunctionPlugin> plugin_loader_;

  // Feedback manager for handling vibration patterns (one physical gamepad -> shared, OCS-side)
  std::shared_ptr<hector_gamepad_plugin_interface::FeedbackManager> feedback_manager_;

  // Config directory relative to package share or absolute path override
  std::string config_directory_;

  // namespace for the operator station
  std::string ocs_namespace_;

  // Double-press window in seconds (ROS param `double_press_window_sec`, default 0.25).
  double double_press_window_sec_;

  // ---- Per-robot state ------------------------------------------------------------------------

  // All built robot controls, keyed by robot namespace (cached for instant switch-back).
  std::map<std::string, std::shared_ptr<RobotControl>> robots_;

  // The currently controlled robot (nullptr only before the first build).
  std::shared_ptr<RobotControl> active_;

  // Deadzone to consider an axis as pressed
  static constexpr float AXIS_DEADZONE = 0.5;

  /**
   * @brief Create the OCS sub-node and the once-only OCS-side entities (config/robot publishers,
   *        feedback manager, joy subscription, switch service, executor), then build and activate
   *        the startup robot from the base node's parameters.
   */
  void setupOcs( const rclcpp::Node::SharedPtr &node );

  /**
   * @brief Build (but do not activate) the control state for a robot namespace.
   *
   * Creates a fresh real node carrying the namespace and the plugin-param overrides loaded from
   * @p plugin_params, constructs the blackboard (or reuses @p reuse_blackboard so soft-e-stop and
   * other runtime values survive a rebuild), the controller orchestrator, adds the node to the
   * executor and loads the config-switches file (which lazily instantiates + initializes plugins).
   * Plugins are NOT activated. On failure the node is removed and nullptr is returned (not cached).
   */
  std::shared_ptr<RobotControl>
  buildControl( const std::string &robot_namespace, const std::string &config_switches_name,
                const std::string &plugin_params,
                std::shared_ptr<hector_gamepad_plugin_interface::Blackboard> reuse_blackboard = nullptr );

  /// Service callback: resolve the target robot, build/rebuild as needed, deactivate the current
  /// robot and activate the target. Empty request fields keep the current value.
  void handleSwitchConfig( const std::shared_ptr<SwitchConfig::Request> request,
                           std::shared_ptr<SwitchConfig::Response> response );

  /// Activate a robot control's last (or default) within-robot config and publish it. Used when
  /// switching robots; bypasses switchConfig()'s same-config early return.
  void activateControl( RobotControl &rc );

  /// Latched publish of the active robot namespace for the UI.
  void publishActiveRobot( const std::string &robot_namespace );

  /**
   * @brief Load the Config Switch File, determines which configs must be loaded
   */
  bool loadConfigSwitchesConfig( RobotControl &rc, const std::string &file_name );

  /**
   * @brief Load the configuration file.
   * @return True if the configuration file was loaded successfully, false otherwise.
   */
  bool loadConfig( RobotControl &rc, const std::string &pkg_name, const std::string &file_name );

  /**
   * @brief Handle the configuration switches.
   * @return True if the configuration switching is in progress and normal button / axis behavior should be ignored
   */
  bool handleConfigurationSwitches( RobotControl &rc, const GamepadInputs &inputs );

  /**
   * @brief Switch the active WITHIN-robot configuration (gamepad-button driven).
   * @return True if switching was successful, false otherwise.
   */
  bool switchConfig( RobotControl &rc, const std::string &config_name );

  /**
   * @brief Initialize the mappings for buttons or axes.
   * @return True if the mappings were initialized successfully, false otherwise.
   */
  bool initButtonMappings( RobotControl &rc, const YAML::Node &config, const std::string &config_name,
                           std::unordered_map<int, ButtonFunctionMapping> &mappings );

  bool initMappings( RobotControl &rc, const YAML::Node &config, const std::string &type,
                     const std::string &config_name,
                     std::unordered_map<int, FunctionMapping> &mappings );

  // Load the named plugin into rc.plugins if not already present. Returns false on failure.
  bool ensurePluginLoaded( RobotControl &rc, const std::string &plugin_name );

  /**
   * @brief Apply a named plugin-param set to one robot's plugins at runtime.
   *
   * Before switching, the current (possibly runtime-modified) values of the active set are
   * snapshotted so they can be restored on reselection. The applied values are then:
   *   - @p reset == true  -> the set's YAML defaults (config/plugin_params/<name>.yaml), or
   *   - @p reset == false -> the snapshotted last values if the set was applied before,
   *                          else the YAML defaults.
   * Only parameters whose plugin is currently loaded (declared) are set; the rest are skipped.
   */
  void applyPluginParamSet( RobotControl &rc, const std::string &name, bool reset );

  /**
   * @brief Activates all plugins present in the given config of the given robot.
   */
  void activatePlugins( RobotControl &rc, const std::string &config_name );

  /**
   * @brief Deactivates all of the given robot's plugins.
   */
  void deactivatePlugins( RobotControl &rc );

  // Synthesize the events needed to bring plugins back to a "no button held" state for double-press buttons before a config switch or shutdown.
  void flushPendingButtonState( RobotControl &rc );

  /**
   * @brief Callback function for the joy topic. Routes to the active robot.
   */
  void joyCallback( sensor_msgs::msg::Joy::SharedPtr msg );

  /**
   * @brief Convert the joy message to gamepad inputs.
   */
  GamepadInputs convertJoyToGamepadInputs( const sensor_msgs::msg::Joy::SharedPtr &msg );

  /**
   * @brief Get the path of a file in a package. Assuming the file is in the config folder.
   */
  std::string getPath( const std::string &pkg_name, const std::string &file_name );
};
} // namespace hector_gamepad_manager

#endif // HECTOR_GAMEPAD_MANAGER_HECTOR_GAMEPAD_MANAGER_HPP
