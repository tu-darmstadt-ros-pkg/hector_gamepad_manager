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

/// A joy message carrying fewer buttons than this is from another source and is dropped
constexpr std::size_t kMinControllerButtons = 15;

// How often a persistent joy-source mismatch is repeated
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
  const rclcpp::QoS qos_profile = rclcpp::QoS( 1 ).reliable().transient_local();
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
  if ( loadConfigSwitchesConfig( config_switches_filename ) && switchConfig( default_config_ ) ) {
    // Configs are immutable after load, so the mapping is published once and latched.
    mapping_publisher_->publish(
        buildGamepadMappingMsg( configs_, config_switch_button_mapping_, default_config_ ) );

    joy_subscription_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, std::bind( &HectorGamepadManager::joyCallback, this, std::placeholders::_1 ) );
  } else {
    RCLCPP_FATAL( node_->get_logger(),
                  "Gamepad manager failed to start (see errors above): not subscribing to joy, "
                  "ALL GAMEPAD INPUT WILL BE IGNORED." );
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

    // Start from empty mappings: two config switch buttons may name the same config, so this can
    // run twice for one entry. configs_ is a std::map, so the reference survives the loads below.
    GamepadConfig &gamepad_config = configs_[file_name];
    gamepad_config = GamepadConfig();
    if ( config["description"] )
      gamepad_config.description = config["description"].as<std::string>();

    if ( !initButtonMappings( config, file_name, gamepad_config.button_mappings ) ||
         !initAxisMappings( config, file_name, gamepad_config.axis_mappings ) ) {
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

std::shared_ptr<HectorGamepadManager::GamepadFunctionPlugin>
HectorGamepadManager::loadPlugin( const std::string &plugin_name )
{
  const auto it = plugins_.find( plugin_name );
  if ( it != plugins_.end() )
    return it->second;
  try {
    std::shared_ptr<GamepadFunctionPlugin> plugin =
        plugin_loader_.createSharedInstance( plugin_name );
    plugin->initializePlugin( node_, plugin_name, blackboard_, feedback_manager_,
                              controller_orchestrator_ );
    plugins_[plugin_name] = plugin;
    RCLCPP_DEBUG( node_->get_logger(), "Loaded plugin: %s", plugin_name.c_str() );
    return plugin;
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( node_->get_logger(), "Failed to load plugin %s: %s", plugin_name.c_str(), e.what() );
    return nullptr;
  }
}

namespace
{
// Read {function, description} from an event node (e.g. on_press). Empty mapping if absent.
ActionMapping readAction( const YAML::Node &node, const std::string &description_fallback = "" )
{
  if ( !node )
    return {};
  return { node["function"].as<std::string>( "" ),
           node["description"].as<std::string>( description_fallback ) };
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
    const std::string description = node["description"].as<std::string>( "" );

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

    mapping.plugin = loadPlugin( plugin_name );
    if ( !mapping.plugin )
      return false;
    mappings[entry.id] = std::move( mapping );
  }
  return true;
}

bool HectorGamepadManager::initAxisMappings( const YAML::Node &config, const std::string &config_name,
                                             std::map<int, FunctionMapping> &mappings )
{
  if ( !config["axes"] ) {
    RCLCPP_ERROR( node_->get_logger(), "No axes found in config file" );
    return false;
  }
  for ( const auto &entry : config["axes"] ) {
    const auto name = entry.first.as<std::string>( "" );
    const int id = axisId( name );
    if ( id < 0 ) {
      RCLCPP_ERROR( node_->get_logger(), "Unknown axis '%s'. Valid names: %s", name.c_str(),
                    axisNameList().c_str() );
      return false;
    }
    const YAML::Node node = entry.second;
    const auto plugin_name = node["plugin"].as<std::string>( "" );
    const auto function = node["function"].as<std::string>( "" );
    // Nothing to bind. Checked before the args are stored, or they would land under a binding id
    // no mapping ever reads.
    if ( plugin_name.empty() || function.empty() )
      continue;

    const std::string binding_id = axisBindingId( config_name, name );
    if ( node["args"] )
      blackboard_->set_from_yaml( node["args"], plugin_name + "_" + binding_id );

    const auto plugin = loadPlugin( plugin_name );
    if ( !plugin )
      return false;
    mappings[id] = { plugin, function, node["description"].as<std::string>( "" ), binding_id };
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

bool HectorGamepadManager::checkJoySource( const sensor_msgs::msg::Joy &msg )
{
  // this function verifies whether the joy msgs is valid e.g. warns if msg from joy_node instead of gamepadnode
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
    return false;
  }

  // Second tell: the trigger convention, which catches a source of the right shape. A trigger
  // resting at +1 is joy_node; game_controller_node rests at 0 and goes negative when pressed, so
  // a positive reading cannot come from it.
  // Indexing is safe: a message of the wrong axis count returned above.
  for ( int id = 0; id < static_cast<int>( kNumAxes ); id++ ) {
    if ( !isTriggerAxis( id ) || msg.axes[id] <= AXIS_DEADZONE )
      continue;
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), kJoySourceReportIntervalMs,
        "Axis '%s' on '%s' reads %.2f, but joy's game_controller_node rests a trigger at 0 and "
        "drives it negative when pressed. This is a different source - most likely joy_node, whose "
        "raw indices address different controls. Launch `game_controller_node` from the joy "
        "package instead (published by:%s).",
        axisName( id ).c_str(), topic, msg.axes[id], publisherNames( *node_, topic ).c_str() );
    return false;
  }
  return true;
}

void HectorGamepadManager::joyCallback( const sensor_msgs::msg::Joy::SharedPtr msg )
{
  if ( !checkJoySource( *msg ) ) {
    // Dropped rather than dispatched: the ids in a message of another layout mean different
    // controls than the config was written against, so acting on it drives whatever happens to
    // share the index - a resting joy_node trigger reads as a fully deflected canonical axis.
    if ( joy_source_ok_ ) {
      // Only on the transition. Nothing after this produces the release for a button that was
      // down when the source went wrong, so it goes out now instead of leaving a plugin held.
      flushPendingButtonState();
      button_trackers_ = {};
      joy_source_ok_ = false;
    }
    return;
  }
  joy_source_ok_ = true;

  const auto inputs = convertJoyToGamepadInputs( *msg );
  // ignore normal button / axis behavior if configuration switching is in progress
  if ( handleConfigurationSwitches( inputs ) )
    return;

  const auto now = node_->now();

  // Backward clock jumps (sim-time replay/reset) count as expired so a press cannot stay buffered
  // forever. Re-read rather than cached: a rising edge restarts the window mid-iteration.
  //
  // A button with no double press bound has no window to wait out at all, so its press is buffered
  // and released again within the iteration that produced it. That is what lets the machine below
  // be the only dispatch path instead of one of two: with a zero-length window it reduces to
  // "press on the rising edge, hold while down, release on the falling edge".
  const auto window_expired = [this, now]( const ButtonFunctionMapping &mapping,
                                           const ButtonTracker &tracker ) {
    if ( !mapping.has_double_press() )
      return true;
    const double elapsed = ( now - tracker.last_press_time ).seconds();
    return elapsed < 0.0 || elapsed >= double_press_window_sec_;
  };

  const GamepadConfig &config = configs_[active_config_];
  for ( const auto &[button_id, mapping] : config.button_mappings ) {
    const bool pressed = inputs.buttons[button_id];
    const std::string &id = mapping.binding_id;
    auto &tracker = button_trackers_[button_id];
    const bool was_pressed = tracker.pressed;
    tracker.pressed = pressed;

    if ( pressed && !was_pressed ) { // rising edge
      if ( tracker.state == PressState::Buffering && !window_expired( mapping, tracker ) ) {
        // Second press within window → double press detected
        tracker.state = PressState::Dispatched;
        mapping.plugin->handlePress( mapping.on_double_press.function, id );
      } else {
        // A stale buffered tap has to go out before last_press_time is overwritten, or the press
        // it holds is lost when no callback fired during the wait window. The button being down
        // again is a new press, so the old one takes its release now.
        if ( tracker.state == PressState::Buffering )
          dispatchBufferedPress( mapping, tracker, false );
        // First press → start waiting for potential second press
        tracker.state = PressState::Buffering;
        tracker.last_press_time = now;
      }
    } else if ( pressed ) {
      // Held — only dispatch hold if press was already dispatched
      if ( tracker.state == PressState::Dispatched ) {
        mapping.plugin->handleHold( mapping.holdFunction(), id );
      }
    } else if ( was_pressed ) { // falling edge
      if ( tracker.state == PressState::Dispatched ) {
        mapping.plugin->handleRelease( mapping.releaseFunction(), id );
        tracker.state = PressState::Idle;
      }
      // While Buffering, keep waiting — the second press can still arrive after the release.
    }

    // The window ran out with no second press: the tap was a single press after all.
    if ( tracker.state == PressState::Buffering && window_expired( mapping, tracker ) )
      dispatchBufferedPress( mapping, tracker, tracker.pressed );
  }

  // Handle axes
  for ( const auto &[axis_id, mapping] : config.axis_mappings ) {
    mapping.plugin->handleAxis( mapping.function_name, mapping.binding_id, inputs.axes[axis_id] );
  }

  // Update all active plugins
  for ( const auto &plugin : active_plugins_ ) { plugin->update(); }
}

void HectorGamepadManager::activatePlugins( const std::string &config_name )
{
  // Every plugin the config binds, whether to a button or an axis, and each activated once even
  // when several bindings share it.
  const auto activate = [this]( const auto &mappings ) {
    for ( const auto &[input_id, mapping] : mappings ) {
      if ( mapping.plugin->isActive() )
        continue;
      mapping.plugin->activate();
      RCLCPP_DEBUG( node_->get_logger(), "Activated plugin: %s",
                    mapping.plugin->getPluginName().c_str() );
      active_plugins_.push_back( mapping.plugin );
    }
  };
  // switchConfig is the only caller and rejects an unknown name before getting here.
  const GamepadConfig &config = configs_.at( config_name );
  activate( config.button_mappings );
  activate( config.axis_mappings );
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

void HectorGamepadManager::dispatchBufferedPress( const ButtonFunctionMapping &mapping,
                                                  ButtonTracker &tracker, const bool still_held )
{
  mapping.plugin->handlePress( mapping.on_press.function, mapping.binding_id );
  if ( still_held ) {
    // The coming messages drive hold and release through the normal path.
    tracker.state = PressState::Dispatched;
    return;
  }
  // Nothing later will produce the release, so it goes out paired with the press.
  mapping.plugin->handleRelease( mapping.releaseFunction(), mapping.binding_id );
  tracker.state = PressState::Idle;
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
    auto &tracker = button_trackers_[button_id];

    switch ( tracker.state ) {
    case PressState::Dispatched:
      // A button still down when the config goes away: the manager detects the edges, so nothing
      // else would ever produce the release that ends the press it already sent.
      mapping.plugin->handleRelease( mapping.releaseFunction(), mapping.binding_id );
      tracker.state = PressState::Idle;
      break;
    case PressState::Buffering:
      // The outgoing config will never see the second press, so the tap resolves to a single one.
      dispatchBufferedPress( mapping, tracker, false );
      break;
    case PressState::Idle:
      break;
    }
  }
}

HectorGamepadManager::GamepadInputs
HectorGamepadManager::convertJoyToGamepadInputs( const sensor_msgs::msg::Joy &msg )
{
  GamepadInputs inputs;
  // Pads report different numbers of buttons and axes (paddles and a touchpad only exist on some),
  // so read out-of-range entries as neutral instead of indexing past the end of the arrays.
  const auto button = [&msg]( const size_t i ) {
    return i < msg.buttons.size() && msg.buttons[i] != 0;
  };
  const auto axis = [&msg]( const size_t i ) { return i < msg.axes.size() ? msg.axes[i] : 0.0f; };

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
  if ( path.extension() != ".yaml" ) {
    path += ".yaml";
  }
  return path.string();
}
} // namespace hector_gamepad_manager
