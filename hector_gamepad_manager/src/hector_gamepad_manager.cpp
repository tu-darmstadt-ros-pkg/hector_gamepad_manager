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
    std::vector<ButtonEntry> entries;
    if ( !collectButtonEntries( config, entries ) )
      return false;
    for ( const auto &entry : entries ) {
      const YAML::Node mapping = entry.node;
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
      config_switch_button_mapping_[entry.id] = { config_name, description };
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
  button_trackers_ = {};
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

bool HectorGamepadManager::collectButtonEntries( const YAML::Node &config,
                                                 std::vector<ButtonEntry> &entries )
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
      if ( id < 0 || isAxisButton( id ) != axis_derived ) {
        RCLCPP_ERROR( node_->get_logger(), "Unknown button '%s' in '%s'. Valid names: %s",
                      name.c_str(), axis_derived ? "axis_buttons" : "buttons",
                      buttonNameList( axis_derived ).c_str() );
        return false;
      }
      entries.push_back( { id, name, entry.second } );
    }
    return true;
  };
  return collect( config["buttons"], false ) && collect( config["axis_buttons"], true );
}

bool HectorGamepadManager::initButtonMappings( const YAML::Node &config,
                                               const std::string &config_name,
                                               std::map<int, ButtonFunctionMapping> &mappings )
{
  std::vector<ButtonEntry> entries;
  if ( !collectButtonEntries( config, entries ) )
    return false;

  for ( const auto &entry : entries ) {
    const YAML::Node node = entry.node;
    if ( !node["plugin"] )
      continue;
    const auto plugin_name = node["plugin"].as<std::string>();
    if ( plugin_name.empty() )
      continue;

    // Per-event format if any on_* key is present. The legacy flat format is the same thing with
    // the press action written straight onto the button, so both read through readAction().
    const bool per_event =
        node["on_press"] || node["on_double_press"] || node["on_hold"] || node["on_release"];
    const YAML::Node press_node = per_event ? node["on_press"] : node;
    const std::string description = node["description"] ? node["description"].as<std::string>() : "";

    ButtonFunctionMapping mapping;
    mapping.on_press = readAction( press_node, description );
    if ( per_event ) {
      mapping.on_double_press = readAction( node["on_double_press"], description );
      mapping.on_hold = readAction( node["on_hold"], description );
      mapping.on_release = readAction( node["on_release"], description );
    }

    // on_press is what a single press dispatches, what the timeout flush replays and what an
    // unset on_hold/on_release falls back to, so a binding without one has nothing to dispatch.
    if ( mapping.on_press.empty() ) {
      RCLCPP_WARN( node_->get_logger(),
                   "Button '%s' in config '%s' has a plugin but no press function ('function', or "
                   "'on_press: {function: ...}' in the per-event format). Skipping.",
                   entry.name.c_str(), config_name.c_str() );
      continue;
    }

    // All events on a button share one args block; per-event args are not distinguishable on the
    // read side. A top-level 'args' wins over one written under 'on_press'.
    mapping.binding_id = buttonBindingId( config_name, entry.name );
    const YAML::Node args = node["args"] ? node["args"] : press_node["args"];
    if ( args )
      blackboard_->set_from_yaml( args, plugin_name + "_" + mapping.binding_id );
    for ( const auto &event_key : { "on_double_press", "on_hold", "on_release" } ) {
      if ( node[event_key] && node[event_key]["args"] ) {
        RCLCPP_WARN( node_->get_logger(),
                     "Per-event args under '%s' on button '%s' are not supported and will be "
                     "ignored. Move them to a top-level 'args:' block.",
                     event_key, entry.name.c_str() );
      }
    }

    if ( !ensurePluginLoaded( plugin_name ) )
      return false;
    mapping.plugin = plugins_[plugin_name];
    mappings[entry.id] = std::move( mapping );
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
        RCLCPP_ERROR( node_->get_logger(), "Unknown axis '%s'. Valid names: %s", name.c_str(),
                      axisNameList().c_str() );
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
  // A handful of comparisons against a callback that already does far more per message, so both
  // tells are checked on every message rather than only the first. publisherNames() runs a graph
  // query, so it sits in the argument lists where the throttle macros only evaluate it on the
  // messages they actually log.
  const char *topic = joy_subscription_->get_topic_name();

  // First tell: the message shape. game_controller_node sizes every message the same way whatever
  // the pad is, so a different count is a different node.
  if ( msg.axes.size() != kNumAxes || msg.buttons.size() < kMinControllerButtons ) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), kJoySourceReportIntervalMs,
        "'%s' carries %zu axes and %zu buttons, but joy's game_controller_node always publishes "
        "%zu axes and at least %zu. This looks like joy_node, which reports raw device-specific "
        "indices, so every button and axis bound in this config addresses the wrong control. "
        "Launch `game_controller_node` from the joy package instead (published by:%s).",
        topic, msg.axes.size(), msg.buttons.size(), kNumAxes, kMinControllerButtons,
        publisherNames( *node_, topic ).c_str() );
    return;
  }

  // Second tell: the trigger convention, which catches a source of the right shape. A trigger
  // resting at +1 is joy_node; game_controller_node rests at 0 and goes negative when pressed, so
  // a positive reading cannot come from it.
  static const int triggers[] = { axisId( "left_trigger" ), axisId( "right_trigger" ) };
  for ( const int id : triggers ) {
    if ( msg.axes[id] <= AXIS_DEADZONE )
      continue;
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), kJoySourceReportIntervalMs,
        "Axis '%s' on '%s' reads %.2f, but joy's game_controller_node rests a trigger at 0 and "
        "drives it negative when pressed. This is a different source - most likely joy_node, whose "
        "raw indices address different controls. Launch `game_controller_node` from the joy "
        "package instead (published by:%s).",
        axisName( id ).c_str(), topic, msg.axes[id], publisherNames( *node_, topic ).c_str() );
    return;
  }
}

void HectorGamepadManager::joyCallback( const sensor_msgs::msg::Joy::SharedPtr msg )
{
  checkJoySource( *msg );
  const auto inputs = convertJoyToGamepadInputs( msg );
  // ignore normal button / axis behavior if configuration switching is in progress
  if ( handleConfigurationSwitches( inputs ) )
    return;

  const auto now = node_->now();

  // Backward clock jumps (sim-time replay/reset) count as expired so a press cannot stay buffered
  // forever. Re-read rather than cached: a rising edge restarts the window mid-iteration.
  const auto window_expired = [this, now]( const ButtonTracker &tracker ) {
    const double elapsed = ( now - tracker.last_press_time ).seconds();
    return elapsed < 0.0 || elapsed >= double_press_window_sec_;
  };

  for ( const auto &[button_id, mapping] : configs_[active_config_].button_mappings ) {
    const bool pressed = inputs.buttons[button_id];
    const std::string &id = mapping.binding_id;
    auto &tracker = button_trackers_[button_id];
    const bool was_pressed = tracker.pressed;
    tracker.pressed = pressed;

    if ( !mapping.has_double_press() ) {
      // No double-press configured → dispatch immediately via handleButton (original behavior)
      mapping.plugin->handleButton( mapping.on_press.function, id, pressed );
      continue;
    }

    // Double-press enabled → buffered dispatch
    if ( pressed && !was_pressed ) { // rising edge
      if ( tracker.awaiting_double_press && !window_expired( tracker ) ) {
        // Second press within window → double press detected
        tracker.awaiting_double_press = false;
        tracker.press_dispatched = true;
        mapping.plugin->handlePress( mapping.on_double_press.function, id );
      } else {
        // Flush a stale buffered tap before overwriting last_press_time, otherwise the original press is silently dropped when no callback fired during the wait window.
        if ( tracker.awaiting_double_press ) {
          mapping.plugin->handlePress( mapping.on_press.function, id );
          mapping.plugin->handleRelease( mapping.releaseFunction(), id );
        }
        // First press → start waiting for potential second press
        tracker.awaiting_double_press = true;
        tracker.last_press_time = now;
        tracker.press_dispatched = false;
      }
    } else if ( pressed ) {
      // Held — only dispatch hold if press was already dispatched
      if ( tracker.press_dispatched ) {
        mapping.plugin->handleHold( mapping.holdFunction(), id );
      }
    } else if ( was_pressed ) { // falling edge
      if ( tracker.press_dispatched ) {
        mapping.plugin->handleRelease( mapping.releaseFunction(), id );
        tracker.press_dispatched = false;
      }
      // If awaiting_double_press, keep waiting — the second press can still arrive after release.
    }

    // Flush a buffered single press whose window has expired without a second press arriving.
    if ( tracker.awaiting_double_press && window_expired( tracker ) ) {
      tracker.awaiting_double_press = false;
      mapping.plugin->handlePress( mapping.on_press.function, id );

      if ( tracker.pressed ) {
        // Still held — let subsequent frames drive hold/release through the normal path.
        tracker.press_dispatched = true;
      } else {
        // Quick tap: pair the delayed press with an immediate release so the plugin doesn't get stuck.
        mapping.plugin->handleRelease( mapping.releaseFunction(), id );
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
  for ( const auto &[button_id, mapping] : config_it->second.button_mappings ) {
    if ( !mapping.has_double_press() )
      continue;

    auto &tracker = button_trackers_[button_id];
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

  // Physical buttons map 1:1, so a pad reporting paddles or a touchpad needs no code change.
  for ( int i = 0; i < static_cast<int>( kVirtualButtonBase ); i++ )
    inputs.buttons[i] = button( i );

  // Axis-derived virtual buttons: each is its own axis deflected past the deadzone in its own
  // direction, both read off the row that also names it.
  const auto &axis_buttons = axisButtons();
  for ( std::size_t i = 0; i < axis_buttons.size(); i++ ) {
    inputs.buttons[kVirtualButtonBase + i] =
        axis_buttons[i].direction * inputs.axes[axis_buttons[i].axis] > AXIS_DEADZONE;
  }
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
