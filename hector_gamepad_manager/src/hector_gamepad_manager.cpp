#include "hector_gamepad_manager/hector_gamepad_manager.hpp"

#include "hector_gamepad_manager/gamepad_buttons.hpp"
#include "hector_gamepad_manager/gamepad_config.hpp"
#include "hector_gamepad_manager/gamepad_mapping_builder.hpp"

#include <filesystem>

namespace hector_gamepad_manager
{
namespace
{
// One-shot feedback pattern fired whenever the active gamepad config changes.
constexpr char kConfigSwitchVibrationId[] = "config_switch_vibration";

// game_controller_node sizes its message once, from SDL_CONTROLLER_AXIS_MAX and
// SDL_CONTROLLER_BUTTON_MAX, so every message carries the same counts whatever the pad is. The
// axis count has always been 6; the button count grew to 21 with the paddles and touchpad in SDL
// 2.0.14, so anything from the earlier 15 upwards counts as the canonical layout. joy_node instead
// sizes per device - an Xbox pad gives 8 axes and 11 buttons - which is what this separates.
constexpr std::size_t kMinControllerButtons = 15;

// How often a persistent joy-source mismatch is repeated, so an operator attaching to the log
// after startup still sees it.
constexpr int kJoySourceReportIntervalMs = 10000;

// Nodes publishing on `topic`, to point at which node to fix. The node's *name* proves nothing -
// a launch file may run either joy executable under any name - so this only locates it.
std::string publisherNames( const rclcpp::Node &node, const char *topic )
{
  std::string names;
  for ( const auto &info : node.get_publishers_info_by_topic( topic ) ) {
    const std::string ns = info.node_namespace();
    names += " " + ( ns == "/" ? "" : ns ) + "/" + info.node_name();
  }
  return names.empty() ? " unknown" : names;
}
} // namespace

HectorGamepadManager::HectorGamepadManager( const rclcpp::Node::SharedPtr &node )
    : node_( node ), plugin_loader_( "hector_gamepad_manager",
                                     "hector_gamepad_plugin_interface::GamepadFunctionPlugin" ),
      blackboard_( std::make_shared<hector_gamepad_plugin_interface::Blackboard>() ),
      feedback_manager_( std::make_shared<hector_gamepad_plugin_interface::FeedbackManager>() )
{
  // declare & get parameters
  node_->declare_parameter<std::string>( "config_name", "athena" );
  node_->declare_parameter<std::string>( "config_directory", "config" );
  node_->declare_parameter<double>( "double_press_window_sec", 0.25 );
  node_->declare_parameter<bool>( "config_switch_vibration_enabled", true );
  const std::string config_switches_filename = node_->get_parameter( "config_name" ).as_string();

  config_directory_ = node_->get_parameter( "config_directory" ).as_string();
  double_press_window_sec_ = node_->get_parameter( "double_press_window_sec" ).as_double();

  // The node is launched into the robot namespace, so all topics below are robot-namespaced.
  // setup config publisher
  rclcpp::QoS qos_profile( 1 );
  qos_profile.reliability( RMW_QOS_POLICY_RELIABILITY_RELIABLE );
  qos_profile.durability( RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL );
  active_config_publisher_ =
      node_->create_publisher<std_msgs::msg::String>( "joy_teleop_profile", qos_profile );
  mapping_publisher_ = node_->create_publisher<hector_gamepad_manager_msgs::msg::GamepadMapping>(
      "joy_mapping", qos_profile );
  feedback_manager_->initialize( node_, "joy_feedback" );
  // Short rumble confirming a config switch on the gamepad (tunable via the
  // config_switch_vibration.* parameters). Pulses shorter than ~0.2 s or much weaker than 0.8
  // are not reliably perceptible on Xbox pads (motor spin-up time).
  hector_gamepad_plugin_interface::VibrationPatternDefaults config_switch_vibration;
  config_switch_vibration.on_durations_sec = { 0.25 };
  config_switch_vibration.off_durations_sec = { 0.0 };
  config_switch_vibration.intensity = 0.8;
  config_switch_vibration.cycle = false;
  feedback_manager_->createVibrationPattern( kConfigSwitchVibrationId, config_switch_vibration );
  controller_orchestrator_ =
      std::make_shared<controller_orchestrator::ControllerOrchestrator>( node_ );
  // load meta switch config and all referenced config files
  if ( loadConfigSwitchesConfig( config_switches_filename ) ) {
    switchConfig( default_config_ );

    // Configs are immutable after load, so the mapping is published once and latched.
    mapping_publisher_->publish(
        buildGamepadMappingMsg( configs_, config_switch_button_mapping_, default_config_ ) );

    joy_subscription_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, std::bind( &HectorGamepadManager::joyCallback, this, std::placeholders::_1 ) );
  }
}

bool HectorGamepadManager::loadConfigSwitchesConfig( const std::string &file_name )
{

  try {
    const YAML::Node config = YAML::LoadFile( getPath( "hector_gamepad_manager", file_name ) );
    std::vector<std::tuple<int, std::string, YAML::Node>> entries;
    if ( !collectButtonEntries( config, entries ) )
      return false;
    for ( const auto &[id, name, mapping] : entries ) {
      auto config_name = mapping["config"].as<std::string>();
      auto pkg_name = mapping["package"].as<std::string>();
      if ( config_name.empty() || pkg_name.empty() )
        continue; // skip empty mappings

      RCLCPP_DEBUG( node_->get_logger(), "Loading config file %s", config_name.c_str() );
      if ( !loadConfig( pkg_name, config_name ) ) {
        RCLCPP_ERROR( node_->get_logger(), "Failed to load config file %s", config_name.c_str() );
        return false;
      }
      std::string description;
      if ( mapping["description"] )
        description = mapping["description"].as<std::string>();
      config_switch_button_mapping_[id] = { config_name, description };
    }
    default_config_ = config["default_config"].as<std::string>();
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( node_->get_logger(), "Error loading Config Switch YAML file: %s", e.what() );
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
    if ( config["description"] )
      configs_[file_name].description = config["description"].as<std::string>();

    if ( !initButtonMappings( config, file_name, configs_[file_name].button_mappings ) ||
         !initAxisMappings( config, file_name, configs_[file_name].axis_mappings ) ) {
      return false;
    }

    return true;
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( node_->get_logger(), "Error loading YAML file: %s", e.what() );
    return false;
  }
}

bool HectorGamepadManager::switchConfig( const std::string &config_name )
{
  if ( configs_.count( config_name ) == 0 ) {
    RCLCPP_ERROR( node_->get_logger(), "Config %s not found. Cannot switch the gamepad config",
                  config_name.c_str() );
    return false;
  }
  if ( config_name == active_config_ )
    return true;
  RCLCPP_DEBUG( node_->get_logger(), "Switching from config %s to config: %s",
                active_config_.c_str(), config_name.c_str() );
  const bool initial_switch = active_config_.empty();
  // Must run before deactivatePlugins() and before active_config_ is reassigned.
  flushPendingButtonState();
  deactivatePlugins();
  button_trackers_.clear();
  active_config_publisher_->publish( std_msgs::msg::String().set__data( config_name ) );
  active_config_ = config_name;
  activatePlugins( config_name );
  // Confirm the switch with a short rumble; skip the initial activation at startup.
  // Read on every switch so the feature can be toggled at runtime via `ros2 param set`.
  if ( !initial_switch && node_->get_parameter( "config_switch_vibration_enabled" ).as_bool() ) {
    feedback_manager_->setPatternActive( kConfigSwitchVibrationId, true );
  }
  return true;
}

bool HectorGamepadManager::ensurePluginLoaded( const std::string &plugin_name )
{
  if ( plugins_.count( plugin_name ) != 0 )
    return true;
  try {
    std::shared_ptr<GamepadFunctionPlugin> plugin =
        plugin_loader_.createSharedInstance( plugin_name );
    plugin->initializePlugin( node_, plugin_name, blackboard_, feedback_manager_,
                              controller_orchestrator_ );
    plugins_[plugin_name] = plugin;
    RCLCPP_DEBUG( node_->get_logger(), "Loaded plugin: %s", plugin_name.c_str() );
    return true;
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( node_->get_logger(), "Failed to load plugin %s: %s", plugin_name.c_str(), e.what() );
    return false;
  }
}

namespace
{
// Read {function, description} from an event node (e.g. on_press). Empty mapping if absent.
ActionMapping readAction( const YAML::Node &node, const std::string &description_fallback = "" )
{
  ActionMapping action;
  if ( !node )
    return action;
  if ( node["function"] )
    action.function = node["function"].as<std::string>();
  if ( node["description"] )
    action.description = node["description"].as<std::string>();
  else
    action.description = description_fallback;
  return action;
}
} // namespace

bool HectorGamepadManager::collectButtonEntries(
    const YAML::Node &config, std::vector<std::tuple<int, std::string, YAML::Node>> &entries )
{
  if ( !config["buttons"] ) {
    RCLCPP_ERROR( node_->get_logger(), "No buttons found in config file" );
    return false;
  }
  // Both sections are name-keyed; they differ only in whether the button is reported by the
  // gamepad or synthesized from a deflected axis, which the section makes explicit.
  const auto collect = [this, &entries]( const YAML::Node &section, const bool axis_derived ) {
    for ( const auto &entry : section ) {
      const auto name = entry.first.as<std::string>( "" );
      const int id = buttonId( name );
      const bool is_axis_button = id >= static_cast<int>( kVirtualButtonBase );
      if ( id < 0 || is_axis_button != axis_derived ) {
        std::string valid_names;
        for ( const auto &known : buttonNames() ) {
          const bool known_is_axis_button =
              buttonId( known ) >= static_cast<int>( kVirtualButtonBase );
          if ( known_is_axis_button == axis_derived )
            valid_names += known + " ";
        }
        RCLCPP_ERROR( node_->get_logger(), "Unknown button '%s' in '%s'. Valid names: %s",
                      name.c_str(), axis_derived ? "axis_buttons" : "buttons", valid_names.c_str() );
        return false;
      }
      entries.emplace_back( id, name, entry.second );
    }
    return true;
  };
  return collect( config["buttons"], false ) && collect( config["axis_buttons"], true );
}

bool HectorGamepadManager::initButtonMappings( const YAML::Node &config,
                                               const std::string &config_name,
                                               std::map<int, ButtonFunctionMapping> &mappings )
{
  std::vector<std::tuple<int, std::string, YAML::Node>> entries;
  if ( !collectButtonEntries( config, entries ) )
    return false;

  for ( const auto &[id, name, mapping] : entries ) {
    if ( !mapping["plugin"] )
      continue;
    auto plugin_name = mapping["plugin"].as<std::string>();
    if ( plugin_name.empty() )
      continue;

    // Detect new format: presence of on_press, on_double_press, on_hold, or on_release sub-keys
    const bool new_format = mapping["on_press"] || mapping["on_double_press"] ||
                            mapping["on_hold"] || mapping["on_release"];
    const std::string &description =
        mapping["description"] ? mapping["description"].as<std::string>() : "";

    ActionMapping on_press, on_double_press, on_hold, on_release;
    const std::string function_id = buttonBindingId( config_name, name );

    if ( new_format ) {
      on_press = readAction( mapping["on_press"], description );
      on_double_press = readAction( mapping["on_double_press"], description );
      on_hold = readAction( mapping["on_hold"], description );
      on_release = readAction( mapping["on_release"], description );

      // on_press is required as the timeout-flush dispatch target and on_hold/on_release fallback.
      if ( on_press.empty() ) {
        RCLCPP_WARN( node_->get_logger(),
                     "Button '%s' in config '%s' has new-format mapping but no on_press "
                     "function. on_press is required (it is the fallback for on_hold/"
                     "on_release and the dispatch target on a single press). Skipping.",
                     name.c_str(), config_name.c_str() );
        continue;
      }

      // All events on a button share one args block; per-event args are not distinguishable on the read side. Top-level wins over on_press/args fallback.
      const std::string blackboard_prefix = plugin_name + "_" + function_id;
      if ( mapping["args"] ) {
        blackboard_->set_from_yaml( mapping["args"], blackboard_prefix );
      } else if ( mapping["on_press"] && mapping["on_press"]["args"] ) {
        blackboard_->set_from_yaml( mapping["on_press"]["args"], blackboard_prefix );
      }
      for ( const auto &event_key : { "on_double_press", "on_hold", "on_release" } ) {
        if ( mapping[event_key] && mapping[event_key]["args"] ) {
          RCLCPP_WARN( node_->get_logger(),
                       "Per-event args under '%s' on button '%s' are not supported and will be "
                       "ignored. Move them to a top-level 'args:' block.",
                       event_key, name.c_str() );
        }
      }
    } else {
      // Legacy flat format: plugin + function at top level → treat as on_press
      if ( !mapping["function"] ) {
        RCLCPP_WARN( node_->get_logger(),
                     "Button '%s' in config '%s' has 'plugin' but no 'function'. Skipping.",
                     name.c_str(), config_name.c_str() );
        continue;
      }
      auto function = mapping["function"].as<std::string>();
      if ( function.empty() )
        continue;
      on_press.function = function;
      if ( mapping["description"] )
        on_press.description = mapping["description"].as<std::string>();
      blackboard_->set_from_yaml( mapping["args"], plugin_name + "_" + function_id );
    }

    if ( !ensurePluginLoaded( plugin_name ) )
      return false;

    mappings[id] = {
        plugins_[plugin_name], on_press, on_double_press, on_hold, on_release, function_id };
  }
  return true;
}

bool HectorGamepadManager::initAxisMappings( const YAML::Node &config, const std::string &config_name,
                                             std::map<int, FunctionMapping> &mappings )
{
  if ( config["axes"] ) {
    for ( const auto &entry : config["axes"] ) {
      const auto name = entry.first.as<std::string>( "" );
      const int id = axisId( name );
      if ( id < 0 ) {
        std::string valid_names;
        for ( const auto &known : axisNames() ) valid_names += known + " ";
        RCLCPP_ERROR( node_->get_logger(), "Unknown axis '%s'. Valid names: %s", name.c_str(),
                      valid_names.c_str() );
        return false;
      }
      const YAML::Node mapping = entry.second;
      if ( !mapping["plugin"] || !mapping["function"] )
        continue;
      auto plugin_name = mapping["plugin"].as<std::string>();
      auto function = mapping["function"].as<std::string>();
      std::string description;
      if ( mapping["description"] )
        description = mapping["description"].as<std::string>();
      const std::string function_id = axisBindingId( config_name, name );
      if ( mapping["args"] ) {
        blackboard_->set_from_yaml( mapping["args"], plugin_name + std::string( "_" ) + function_id );
      }

      if ( !plugin_name.empty() && !function.empty() ) {
        if ( !ensurePluginLoaded( plugin_name ) )
          return false;
        mappings[id] = { plugins_[plugin_name], function, description, function_id };
      }
    }
  } else {
    RCLCPP_ERROR( node_->get_logger(), "No axes found in config file" );
    return false;
  }
  return true;
}

bool HectorGamepadManager::handleConfigurationSwitches( const GamepadInputs &inputs )
{

  // test if a button is pressed that is mapped to a config switch
  for ( size_t i = 0; i < config_switch_button_mapping_.size(); i++ ) {
    if ( inputs.buttons[i] && !config_switch_button_mapping_[i].config.empty() ) {
      switchConfig( config_switch_button_mapping_[i].config );
      return true;
    }
  }
  return false;
}

void HectorGamepadManager::checkJoySource( const sensor_msgs::msg::Joy &msg )
{
  // Two size comparisons, against a callback that already allocates a binding-id string per bound
  // button, so this is affordable on every message rather than only the first.
  if ( msg.axes.size() == kNumAxes && msg.buttons.size() >= kMinControllerButtons )
    return;

  // publisherNames() runs a graph query, so it sits in the argument list where the throttle macro
  // only evaluates it on the messages it actually logs.
  RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), kJoySourceReportIntervalMs,
      "'%s' carries %zu axes and %zu buttons, but joy's game_controller_node always publishes %zu "
      "axes and at least %zu. This looks like joy_node, which reports raw device-specific indices, "
      "so every button and axis bound in this config addresses the wrong control. Launch "
      "`game_controller_node` from the joy package instead (published by:%s).",
      joy_subscription_->get_topic_name(), msg.axes.size(), msg.buttons.size(), kNumAxes,
      kMinControllerButtons, publisherNames( *node_, joy_subscription_->get_topic_name() ).c_str() );
}

void HectorGamepadManager::joyCallback( const sensor_msgs::msg::Joy::SharedPtr msg )
{
  checkJoySource( *msg );
  const auto inputs = convertJoyToGamepadInputs( msg );
  // ignore normal button / axis behavior if configuration switching is in progress
  if ( handleConfigurationSwitches( inputs ) )
    return;

  const auto now = node_->now();

  // Handle buttons with double-press detection
  for ( const auto &[button_id, mapping] : configs_[active_config_].button_mappings ) {
    const bool pressed = inputs.buttons[button_id];
    const std::string &id = mapping.binding_id;
    auto &tracker = button_trackers_[button_id];
    const bool was_pressed = tracker.pressed;

    if ( !mapping.has_double_press() ) {
      // No double-press configured → dispatch immediately via handleButton (original behavior)
      const std::string &function = mapping.on_press.function;
      mapping.plugin->handleButton( function, id, pressed );
    } else {
      // Double-press enabled → buffered dispatch
      const bool rising_edge = pressed && !was_pressed;
      const bool falling_edge = !pressed && was_pressed;

      // Backward clock jumps (sim-time replay/reset) are treated as "window expired" so the press doesn't stay buffered forever.
      const double raw_elapsed = ( now - tracker.last_press_time ).seconds();
      const bool window_expired = raw_elapsed < 0.0 || raw_elapsed >= double_press_window_sec_;

      if ( rising_edge ) {
        if ( tracker.awaiting_double_press && !window_expired ) {
          // Second press within window → double press detected
          tracker.awaiting_double_press = false;
          tracker.press_dispatched = true;
          mapping.plugin->handlePress( mapping.on_double_press.function, id );
        } else {
          // Flush a stale buffered tap before overwriting last_press_time, otherwise the original press is silently dropped when no callback fired during the wait window.
          if ( tracker.awaiting_double_press && window_expired ) {
            mapping.plugin->handlePress( mapping.on_press.function, id );
            mapping.plugin->handleRelease( mapping.releaseFunction(), id );
          }
          // First press → start waiting for potential second press
          tracker.awaiting_double_press = true;
          tracker.last_press_time = now;
          tracker.press_dispatched = false;
        }
      } else if ( pressed && was_pressed ) {
        // Held — only dispatch hold if press was already dispatched
        if ( tracker.press_dispatched ) {
          mapping.plugin->handleHold( mapping.holdFunction(), id );
        }
      } else if ( falling_edge ) {
        if ( tracker.press_dispatched ) {
          mapping.plugin->handleRelease( mapping.releaseFunction(), id );
          tracker.press_dispatched = false;
        }
        // If awaiting_double_press, keep waiting — the second press can still arrive after release.
      }
    }

    tracker.pressed = pressed;
  }

  // Flush buffered single presses whose double-press window has expired.
  for ( const auto &[button_id, mapping] : configs_[active_config_].button_mappings ) {
    if ( !mapping.has_double_press() )
      continue;

    auto &tracker = button_trackers_[button_id];
    const double raw_elapsed = ( now - tracker.last_press_time ).seconds();
    const bool window_expired = raw_elapsed < 0.0 || raw_elapsed >= double_press_window_sec_;
    if ( tracker.awaiting_double_press && window_expired ) {
      tracker.awaiting_double_press = false;
      mapping.plugin->handlePress( mapping.on_press.function, mapping.binding_id );

      if ( tracker.pressed ) {
        // Still held — let subsequent frames drive hold/release through the normal path.
        tracker.press_dispatched = true;
      } else {
        // Quick tap: pair the delayed press with an immediate release so the plugin doesn't get stuck.
        mapping.plugin->handleRelease( mapping.releaseFunction(), mapping.binding_id );
        tracker.press_dispatched = false;
      }
    }
  }

  // Handle axes
  for ( const auto &[axis_id, mapping] : configs_[active_config_].axis_mappings ) {
    mapping.plugin->handleAxis( mapping.function_name, mapping.binding_id, inputs.axes[axis_id] );
  }

  // Update all active plugins
  for ( const auto &plugin : active_plugins_ ) { plugin->update(); }
}

void HectorGamepadManager::activatePlugins( const std::string &config_name )
{
  // activate all  plugins present in the button_mappings_ and axis_mappings_ of the given config
  if ( configs_.count( config_name ) == 0 ) {
    RCLCPP_ERROR( node_->get_logger(), "Config %s not found. Cannot activate the gamepad config",
                  config_name.c_str() );
    return;
  }
  // activate all plugins present in the button_mappings_
  for ( const auto &button_mapping : configs_[config_name].button_mappings ) {
    if ( !button_mapping.second.plugin->isActive() ) {
      button_mapping.second.plugin->activate();
      RCLCPP_DEBUG( node_->get_logger(), "Activated plugin: %s",
                    button_mapping.second.plugin->getPluginName().c_str() );
      active_plugins_.push_back( button_mapping.second.plugin );
    }
  }
  // activate all plugins present in the axis_mappings_
  for ( const auto &axis_mapping : configs_[config_name].axis_mappings ) {
    if ( !axis_mapping.second.plugin->isActive() ) {
      axis_mapping.second.plugin->activate();
      RCLCPP_DEBUG( node_->get_logger(), "Activated plugin: %s",
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
      RCLCPP_DEBUG( node_->get_logger(), "Deactivated plugin: %s", plugin.first.c_str() );
    }
  }
  active_plugins_.clear();
}

void HectorGamepadManager::flushPendingButtonState()
{
  // Operates on the OUTGOING config — must run before active_config_ is reassigned.
  if ( active_config_.empty() )
    return;
  auto config_it = configs_.find( active_config_ );
  if ( config_it == configs_.end() )
    return;
  const auto &button_mappings = config_it->second.button_mappings;

  for ( auto &[button_id, tracker] : button_trackers_ ) {
    auto mapping_it = button_mappings.find( button_id );
    if ( mapping_it == button_mappings.end() )
      continue;
    const auto &mapping = mapping_it->second;
    if ( !mapping.has_double_press() )
      continue;

    const std::string &id = mapping.binding_id;

    if ( tracker.press_dispatched ) {
      mapping.plugin->handleRelease( mapping.releaseFunction(), id );
      tracker.press_dispatched = false;
    } else if ( tracker.awaiting_double_press ) {
      // Emit the same press+release pair the timeout-quick-tap path would have produced.
      mapping.plugin->handlePress( mapping.on_press.function, id );
      mapping.plugin->handleRelease( mapping.releaseFunction(), id );
    }
    tracker.awaiting_double_press = false;
  }
}

HectorGamepadManager::GamepadInputs
HectorGamepadManager::convertJoyToGamepadInputs( const sensor_msgs::msg::Joy::SharedPtr &msg )
{
  GamepadInputs inputs;
  // Pads report different numbers of buttons and axes (paddles and a touchpad only exist on some),
  // so read out-of-range entries as neutral instead of indexing past the end of the arrays.
  const auto button = [&msg]( const size_t i ) {
    return i < msg->buttons.size() && msg->buttons[i] != 0;
  };
  const auto axis = [&msg]( const size_t i ) { return i < msg->axes.size() ? msg->axes[i] : 0.0f; };

  // Axes, in SDL GameController order. Sticks pass through; triggers are flipped to their
  // canonical 0..1 range - see isTriggerAxis().
  for ( int i = 0; i < static_cast<int>( kNumAxes ); i++ )
    inputs.axes[i] = isTriggerAxis( i ) ? -axis( i ) : axis( i );

  // A trigger outside 0..1 means the joy source is joy_node, whose raw triggers idle at +1 and so
  // land here as a permanently held trigger. Warn instead of accommodating it, which would break
  // pads that report the layout correctly.
  for ( int i = 0; i < static_cast<int>( kNumAxes ); i++ ) {
    if ( isTriggerAxis( i ) && inputs.axes[i] < -AXIS_DEADZONE ) {
      RCLCPP_WARN_THROTTLE( node_->get_logger(), *node_->get_clock(), 10000,
                            "Axis '%s' reads %.2f, outside its 0..1 range. The manager expects "
                            "joy's game_controller_node; joy_node publishes a different layout.",
                            axisName( i ).c_str(), inputs.axes[i] );
    }
  }

  // Physical buttons map 1:1, so a pad reporting paddles or a touchpad needs no code change.
  for ( int i = 0; i < static_cast<int>( kVirtualButtonBase ); i++ )
    inputs.buttons[i] = button( i );

  // Axis-derived virtual buttons. Offsets must match axisButtonIds(). The d-pad is not among
  // them: SDL reports it as four real buttons.
  inputs.buttons[kVirtualButtonBase + 0] = inputs.axes[0] > AXIS_DEADZONE;  // left_stick_left
  inputs.buttons[kVirtualButtonBase + 1] = inputs.axes[0] < -AXIS_DEADZONE; // left_stick_right
  inputs.buttons[kVirtualButtonBase + 2] = inputs.axes[1] > AXIS_DEADZONE;  // left_stick_up
  inputs.buttons[kVirtualButtonBase + 3] = inputs.axes[1] < -AXIS_DEADZONE; // left_stick_down
  inputs.buttons[kVirtualButtonBase + 4] = inputs.axes[4] > AXIS_DEADZONE;  // left_trigger
  inputs.buttons[kVirtualButtonBase + 5] = inputs.axes[2] > AXIS_DEADZONE;  // right_stick_left
  inputs.buttons[kVirtualButtonBase + 6] = inputs.axes[2] < -AXIS_DEADZONE; // right_stick_right
  inputs.buttons[kVirtualButtonBase + 7] = inputs.axes[3] > AXIS_DEADZONE;  // right_stick_up
  inputs.buttons[kVirtualButtonBase + 8] = inputs.axes[3] < -AXIS_DEADZONE; // right_stick_down
  inputs.buttons[kVirtualButtonBase + 9] = inputs.axes[5] > AXIS_DEADZONE;  // right_trigger
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
