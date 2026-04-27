#include "hector_gamepad_manager/hector_gamepad_manager.hpp"

#include <filesystem>

namespace hector_gamepad_manager
{
HectorGamepadManager::HectorGamepadManager( const rclcpp::Node::SharedPtr &node )
    : plugin_loader_( "hector_gamepad_manager",
                      "hector_gamepad_plugin_interface::GamepadFunctionPlugin" ),
      blackboard_( std::make_shared<hector_gamepad_plugin_interface::Blackboard>() ),
      feedback_manager_( std::make_shared<hector_gamepad_plugin_interface::FeedbackManager>() )
{
  // declare & get parameters
  node->declare_parameter<std::string>( "config_name", "athena" );
  node->declare_parameter<std::string>( "config_directory", "config" );
  node->declare_parameter<std::string>( "robot_namespace", "athena" );
  node->declare_parameter<std::string>( "ocs_namespace", "ocs" );
  node->declare_parameter<double>( "double_press_window_sec", 0.25 );
  const std::string config_switches_filename = node->get_parameter( "config_name" ).as_string();

  robot_namespace_ = node->get_parameter( "robot_namespace" ).as_string();
  ocs_namespace_ = node->get_parameter( "ocs_namespace" ).as_string();
  config_directory_ = node->get_parameter( "config_directory" ).as_string();
  double_press_window_sec_ = node->get_parameter( "double_press_window_sec" ).as_double();

  // create subnodes: one for the OCS and one for the robot
  ocs_ns_node_ = node->create_sub_node( ocs_namespace_ );
  robot_ns_node_ = node->create_sub_node( robot_namespace_ );

  // setup config publisher
  rclcpp::QoS qos_profile( 1 );
  qos_profile.reliability( RMW_QOS_POLICY_RELIABILITY_RELIABLE );
  qos_profile.durability( RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL );
  active_config_publisher_ =
      ocs_ns_node_->create_publisher<std_msgs::msg::String>( "joy_teleop_profile", qos_profile );
  feedback_manager_->initialize( ocs_ns_node_ );
  controller_orchestrator_ =
      std::make_shared<controller_orchestrator::ControllerOrchestrator>( robot_ns_node_ );
  // load meta switch config and all referenced config files
  if ( loadConfigSwitchesConfig( config_switches_filename ) ) {
    switchConfig( default_config_ );

    joy_subscription_ = ocs_ns_node_->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, std::bind( &HectorGamepadManager::joyCallback, this, std::placeholders::_1 ) );
  }
}

bool HectorGamepadManager::loadConfigSwitchesConfig( const std::string &file_name )
{

  try {
    const YAML::Node config = YAML::LoadFile( getPath( "hector_gamepad_manager", file_name ) );
    for ( const auto &entry : config["buttons"] ) {
      const int id = entry.first.as<int>();
      YAML::Node mapping = entry.second;
      auto config_name = mapping["config"].as<std::string>();
      auto pkg_name = mapping["package"].as<std::string>();
      if ( config_name.empty() || pkg_name.empty() )
        continue; // skip empty mappings

      RCLCPP_DEBUG( ocs_ns_node_->get_logger(), "Loading config file %s", config_name.c_str() );
      if ( !loadConfig( pkg_name, config_name ) ) {
        RCLCPP_ERROR( ocs_ns_node_->get_logger(), "Failed to load config file %s",
                      config_name.c_str() );
        return false;
      }
      config_switch_button_mapping_[id] = config_name;
    }
    default_config_ = config["default_config"].as<std::string>();
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( ocs_ns_node_->get_logger(), "Error loading Config Switch YAML file: %s", e.what() );
    return false;
  }
  return true;
}

bool HectorGamepadManager::loadConfig( const std::string &pkg_name, const std::string &file_name )
{
  try {
    const YAML::Node config = YAML::LoadFile( getPath( pkg_name, file_name ) );

    // Add empty mappings for the filename
    configs_[file_name] = GamepadConfig();

    if ( !initButtonMappings( config, file_name, configs_[file_name].button_mappings ) ||
         !initMappings( config, "axes", file_name, configs_[file_name].axis_mappings ) ) {
      return false;
    }

    return true;
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( ocs_ns_node_->get_logger(), "Error loading YAML file: %s", e.what() );
    return false;
  }
}

bool HectorGamepadManager::switchConfig( const std::string &config_name )
{
  if ( configs_.count( config_name ) == 0 ) {
    RCLCPP_ERROR( ocs_ns_node_->get_logger(),
                  "Config %s not found. Cannot switch the gamepad config", config_name.c_str() );
    return false;
  }
  if ( config_name == active_config_ )
    return true;
  RCLCPP_DEBUG( ocs_ns_node_->get_logger(), "Switching from config %s to config: %s",
                active_config_.c_str(), config_name.c_str() );
  deactivatePlugins();
  button_trackers_.clear();
  active_config_publisher_->publish( std_msgs::msg::String().set__data( config_name ) );
  active_config_ = config_name;
  activatePlugins( config_name );
  return true;
}

bool HectorGamepadManager::ensurePluginLoaded( const std::string &plugin_name )
{
  if ( plugins_.count( plugin_name ) != 0 )
    return true;
  try {
    std::shared_ptr<GamepadFunctionPlugin> plugin =
        plugin_loader_.createSharedInstance( plugin_name );
    plugin->initializePlugin( robot_ns_node_, ocs_ns_node_, plugin_name, blackboard_,
                              feedback_manager_, controller_orchestrator_ );
    plugins_[plugin_name] = plugin;
    RCLCPP_DEBUG( ocs_ns_node_->get_logger(), "Loaded plugin: %s", plugin_name.c_str() );
    return true;
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( ocs_ns_node_->get_logger(), "Failed to load plugin %s: %s", plugin_name.c_str(),
                  e.what() );
    return false;
  }
}

bool HectorGamepadManager::initButtonMappings( const YAML::Node &config,
                                               const std::string &config_name,
                                               std::unordered_map<int, ButtonFunctionMapping> &mappings )
{
  if ( !config["buttons"] ) {
    RCLCPP_ERROR( ocs_ns_node_->get_logger(), "No buttons found in config file" );
    return false;
  }

  for ( const auto &entry : config["buttons"] ) {
    const int id = entry.first.as<int>();
    const YAML::Node mapping = entry.second;

    if ( !mapping["plugin"] )
      continue;
    auto plugin_name = mapping["plugin"].as<std::string>();
    if ( plugin_name.empty() )
      continue;

    // Detect new format: presence of on_press, on_double_press, on_hold, or on_release sub-keys
    const bool new_format = mapping["on_press"] || mapping["on_double_press"] ||
                            mapping["on_hold"] || mapping["on_release"];

    std::string on_press, on_double_press, on_hold, on_release;
    const std::string function_id = config_name + "_" + std::to_string( id );

    if ( new_format ) {
      if ( mapping["on_press"] && mapping["on_press"]["function"] )
        on_press = mapping["on_press"]["function"].as<std::string>();
      if ( mapping["on_double_press"] && mapping["on_double_press"]["function"] )
        on_double_press = mapping["on_double_press"]["function"].as<std::string>();
      if ( mapping["on_hold"] && mapping["on_hold"]["function"] )
        on_hold = mapping["on_hold"]["function"].as<std::string>();
      if ( mapping["on_release"] && mapping["on_release"]["function"] )
        on_release = mapping["on_release"]["function"].as<std::string>();

      // on_press is required: it is the dispatch target for the timeout-flush path
      // (and the fallback for on_hold/on_release). Without it, a single press of a
      // double-press-configured button would dispatch handlePress/handleRelease with
      // an empty function name.
      if ( on_press.empty() ) {
        RCLCPP_WARN( ocs_ns_node_->get_logger(),
                     "Button %d in config '%s' has new-format mapping but no on_press "
                     "function. on_press is required (it is the fallback for on_hold/"
                     "on_release and the dispatch target on a single press). Skipping.",
                     id, config_name.c_str() );
        continue;
      }

      // Args are stored under one shared blackboard prefix per button, matching the legacy
      // format that built-in plugins read via getConfigValueOr(id, key). All events on a button
      // therefore share one args block — supplied either at top level (`args:`) or, as a
      // convenience, under `on_press: { args: ... }`. Per-event args are not currently
      // distinguishable on the read side; if both are given, top-level wins.
      const std::string blackboard_prefix = plugin_name + "_" + function_id;
      if ( mapping["args"] ) {
        blackboard_->set_from_yaml( mapping["args"], blackboard_prefix );
      } else if ( mapping["on_press"] && mapping["on_press"]["args"] ) {
        blackboard_->set_from_yaml( mapping["on_press"]["args"], blackboard_prefix );
      }
      for ( const auto &event_key : { "on_double_press", "on_hold", "on_release" } ) {
        if ( mapping[event_key] && mapping[event_key]["args"] ) {
          RCLCPP_WARN( ocs_ns_node_->get_logger(),
                       "Per-event args under '%s' on button %d are not supported and will be "
                       "ignored. Move them to a top-level 'args:' block.",
                       event_key, id );
        }
      }
    } else {
      // Legacy flat format: plugin + function at top level → treat as on_press
      if ( !mapping["function"] ) {
        RCLCPP_WARN( ocs_ns_node_->get_logger(),
                     "Button %d in config '%s' has 'plugin' but no 'function'. Skipping.", id,
                     config_name.c_str() );
        continue;
      }
      auto function = mapping["function"].as<std::string>();
      if ( function.empty() )
        continue;
      on_press = function;
      blackboard_->set_from_yaml( mapping["args"], plugin_name + "_" + function_id );
    }

    if ( !ensurePluginLoaded( plugin_name ) )
      return false;

    mappings[id] = { plugins_[plugin_name], on_press, on_double_press, on_hold, on_release };
  }
  return true;
}

bool HectorGamepadManager::initMappings( const YAML::Node &config, const std::string &type,
                                         const std::string &config_name,
                                         std::unordered_map<int, FunctionMapping> &mappings )
{
  if ( config[type] ) {
    for ( const auto &entry : config[type] ) {
      int id = entry.first.as<int>();
      const YAML::Node mapping = entry.second;
      if ( !mapping["plugin"] || !mapping["function"] )
        continue;
      auto plugin_name = mapping["plugin"].as<std::string>();
      auto function = mapping["function"].as<std::string>();
      const std::string function_id = config_name + "_" + std::to_string( id );
      blackboard_->set_from_yaml( mapping["args"], plugin_name + std::string( "_" ) + function_id );

      if ( !plugin_name.empty() && !function.empty() ) {
        if ( !ensurePluginLoaded( plugin_name ) )
          return false;
        mappings[id] = { plugins_[plugin_name], function };
      }
    }
  } else {
    RCLCPP_ERROR( ocs_ns_node_->get_logger(), "No %s found in config file", type.c_str() );
    return false;
  }
  return true;
}

bool HectorGamepadManager::handleConfigurationSwitches( const GamepadInputs &inputs )
{

  // test if a button is pressed that is mapped to a config switch
  for ( size_t i = 0; i < config_switch_button_mapping_.size(); i++ ) {
    if ( inputs.buttons[i] && !config_switch_button_mapping_[i].empty() ) {
      switchConfig( config_switch_button_mapping_[i] );
      return true;
    }
  }
  return false;
}

void HectorGamepadManager::joyCallback( const sensor_msgs::msg::Joy::SharedPtr msg )
{
  const auto inputs = convertJoyToGamepadInputs( msg );
  // ignore normal button / axis behavior if configuration switching is in progress
  if ( handleConfigurationSwitches( inputs ) )
    return;

  const auto now = ocs_ns_node_->now();

  // Handle buttons with double-press detection
  for ( const auto &[button_id, mapping] : configs_[active_config_].button_mappings ) {
    const bool pressed = inputs.buttons[button_id];
    const std::string id = active_config_ + "_" + std::to_string( button_id );
    auto &tracker = button_trackers_[button_id];
    const bool was_pressed = tracker.pressed;

    if ( !mapping.hasDoublePress() ) {
      // No double-press configured → dispatch immediately via handleButton (original behavior)
      const std::string &function = mapping.on_press;
      mapping.plugin->handleButton( function, id, pressed );
    } else {
      // Double-press enabled → buffered dispatch
      const bool rising_edge = pressed && !was_pressed;
      const bool falling_edge = !pressed && was_pressed;

      if ( rising_edge ) {
        if ( tracker.awaiting_double_press &&
             ( now - tracker.last_press_time ).seconds() < double_press_window_sec_ ) {
          // Second press within window → double press detected
          tracker.awaiting_double_press = false;
          tracker.press_dispatched = true;
          mapping.plugin->handlePress( mapping.on_double_press, id );
        } else {
          // First press → start waiting for potential second press
          tracker.awaiting_double_press = true;
          tracker.last_press_time = now;
          tracker.press_dispatched = false;
        }
      } else if ( pressed && was_pressed ) {
        // Held — only dispatch hold if press was already dispatched
        if ( tracker.press_dispatched ) {
          const std::string &hold_fn = mapping.on_hold.empty() ? mapping.on_press : mapping.on_hold;
          mapping.plugin->handleHold( hold_fn, id );
        }
      } else if ( falling_edge ) {
        if ( tracker.press_dispatched ) {
          const std::string &release_fn =
              mapping.on_release.empty() ? mapping.on_press : mapping.on_release;
          mapping.plugin->handleRelease( release_fn, id );
          tracker.press_dispatched = false;
        }
        // Note: if awaiting_double_press, we keep waiting — the second press
        // can arrive after the button is released
      }
    }

    tracker.pressed = pressed;
  }

  // Check for double-press timeouts: if we waited long enough without a second press,
  // dispatch the buffered single press
  for ( const auto &[button_id, mapping] : configs_[active_config_].button_mappings ) {
    if ( !mapping.hasDoublePress() )
      continue;

    auto &tracker = button_trackers_[button_id];
    if ( tracker.awaiting_double_press &&
         ( now - tracker.last_press_time ).seconds() >= double_press_window_sec_ ) {
      tracker.awaiting_double_press = false;
      const std::string id = active_config_ + "_" + std::to_string( button_id );
      mapping.plugin->handlePress( mapping.on_press, id );

      if ( tracker.pressed ) {
        // Still held — let subsequent frames drive hold/release through the normal path.
        tracker.press_dispatched = true;
      } else {
        // Quick tap: button was already released while we waited. Pair the delayed press with
        // an immediate release so plugins that toggle state (velocity on/off, etc.) don't get
        // stuck with a press that never closes.
        const std::string &release_fn =
            mapping.on_release.empty() ? mapping.on_press : mapping.on_release;
        mapping.plugin->handleRelease( release_fn, id );
        tracker.press_dispatched = false;
      }
    }
  }

  // Handle axes
  for ( const auto &axis_mapping : configs_[active_config_].axis_mappings ) {
    const float value = inputs.axes[axis_mapping.first];
    const auto &action = axis_mapping.second;
    const std::string id = active_config_ + "_" + std::to_string( axis_mapping.first );
    axis_mapping.second.plugin->handleAxis( action.function_name, id, value );
  }

  // Update all active plugins
  for ( const auto &plugin : active_plugins_ ) { plugin->update(); }
}

void HectorGamepadManager::activatePlugins( const std::string &config_name )
{
  // activate all  plugins present in the button_mappings_ and axis_mappings_ of the given config
  if ( configs_.count( config_name ) == 0 ) {
    RCLCPP_ERROR( ocs_ns_node_->get_logger(),
                  "Config %s not found. Cannot activate the gamepad config", config_name.c_str() );
    return;
  }
  // activate all plugins present in the button_mappings_
  for ( const auto &button_mapping : configs_[config_name].button_mappings ) {
    if ( !button_mapping.second.plugin->isActive() ) {
      button_mapping.second.plugin->activate();
      RCLCPP_DEBUG( ocs_ns_node_->get_logger(), "Activated plugin: %s",
                    button_mapping.second.plugin->getPluginName().c_str() );
      active_plugins_.push_back( button_mapping.second.plugin );
    }
  }
  // activate all plugins present in the axis_mappings_
  for ( const auto &axis_mapping : configs_[config_name].axis_mappings ) {
    if ( !axis_mapping.second.plugin->isActive() ) {
      axis_mapping.second.plugin->activate();
      RCLCPP_DEBUG( ocs_ns_node_->get_logger(), "Activated plugin: %s",
                    axis_mapping.second.plugin->getPluginName().c_str() );
      active_plugins_.push_back( axis_mapping.second.plugin );
    }
  }
}

void HectorGamepadManager::deactivatePlugins()
{
  for ( const auto &plugin : plugins_ ) {
    if ( plugin.second->isActive() ) {
      plugin.second->deactivate();
      RCLCPP_DEBUG( ocs_ns_node_->get_logger(), "Deactivated plugin: %s", plugin.first.c_str() );
    }
  }
  active_plugins_.clear();
}

HectorGamepadManager::GamepadInputs
HectorGamepadManager::convertJoyToGamepadInputs( const sensor_msgs::msg::Joy::SharedPtr &msg )
{
  GamepadInputs inputs;
  // Axes
  inputs.axes[0] = msg->axes[0];                    // Left joystick left/right
  inputs.axes[1] = msg->axes[1];                    // Left joystick up/down
  inputs.axes[2] = -0.5f * ( msg->axes[2] - 1.0f ); // LT: Change range from [1, -1] to [0, 1]
  inputs.axes[3] = msg->axes[3];                    // Right joystick left/right
  inputs.axes[4] = msg->axes[4];                    // Right joystick up/down
  inputs.axes[5] = -0.5f * ( msg->axes[5] - 1.0f ); // RT: Change range from [1, -1] to [0, 1]
  inputs.axes[6] = msg->axes[6];                    // Cross left/right
  inputs.axes[7] = msg->axes[7];                    // Cross up/down

  // Buttons
  inputs.buttons[0] = msg->buttons[0];   // Button A
  inputs.buttons[1] = msg->buttons[1];   // Button B
  inputs.buttons[2] = msg->buttons[2];   // Button X
  inputs.buttons[3] = msg->buttons[3];   // Button Y
  inputs.buttons[4] = msg->buttons[4];   // Button LB
  inputs.buttons[5] = msg->buttons[5];   // Button RB
  inputs.buttons[6] = msg->buttons[6];   // Button Back
  inputs.buttons[7] = msg->buttons[7];   // Button Start
  inputs.buttons[8] = msg->buttons[8];   // Button Guide -> Reserved for config switches
  inputs.buttons[9] = msg->buttons[9];   // Left joystick pressed
  inputs.buttons[10] = msg->buttons[10]; // Right joystick pressed
  inputs.buttons[11] = inputs.axes[0] > AXIS_DEADZONE;  // Left joystick left
  inputs.buttons[12] = inputs.axes[0] < -AXIS_DEADZONE; // Left joystick right
  inputs.buttons[13] = inputs.axes[1] > AXIS_DEADZONE;  // Left joystick up
  inputs.buttons[14] = inputs.axes[1] < -AXIS_DEADZONE; // Left joystick down
  inputs.buttons[15] = inputs.axes[2] > AXIS_DEADZONE;  // LT button
  inputs.buttons[16] = inputs.axes[3] > AXIS_DEADZONE;  // Right joystick left
  inputs.buttons[17] = inputs.axes[3] < -AXIS_DEADZONE; // Right joystick right
  inputs.buttons[18] = inputs.axes[4] > AXIS_DEADZONE;  // Right joystick up
  inputs.buttons[19] = inputs.axes[4] < -AXIS_DEADZONE; // Right joystick down
  inputs.buttons[20] = inputs.axes[5] > AXIS_DEADZONE;  // RT button
  inputs.buttons[21] = inputs.axes[6] == 1.0f;          // Cross left
  inputs.buttons[22] = inputs.axes[6] == -1.0f;         // Cross right
  inputs.buttons[23] = inputs.axes[7] == 1.0f;          // Cross up
  inputs.buttons[24] = inputs.axes[7] == -1.0f;         // Cross down
  return inputs;
}

std::string HectorGamepadManager::getPath( const std::string &pkg_name, const std::string &file_name )
{
  std::filesystem::path path;
  std::filesystem::path config_dir( config_directory_ );
  if ( config_dir.is_absolute() ) {
    path = config_dir / file_name;
  } else {
    const auto package_path = ament_index_cpp::get_package_share_directory( pkg_name );
    path = std::filesystem::path( package_path ) / config_directory_ / file_name;
  }
  if ( file_name.find( ".yaml" ) == std::string::npos ) {
    path += ".yaml";
  }
  return path.string();
}
} // namespace hector_gamepad_manager
