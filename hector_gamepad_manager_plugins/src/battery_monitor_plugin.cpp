#include "hector_gamepad_manager_plugins/battery_monitor_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <ros_babel_fish/messages/array_message.hpp>
#include <ros_babel_fish/method_invoke_helpers.hpp>
#include <sstream>
#include <type_traits>

namespace hector_gamepad_manager_plugins
{

namespace
{
// Helper: Traverses the message path and calls 'visitor' on every numeric value found.
template<typename Visitor>
void checkPath( const ros_babel_fish::CompoundMessage &root, const std::string &path,
                Visitor check_func, const rclcpp::Logger &logger,
                const rclcpp::Clock::SharedPtr &clock )
{
  const ros_babel_fish::Message *curr = &root;
  std::stringstream ss( path );
  std::string token;

  // 1. Traverse Path
  while ( std::getline( ss, token, '.' ) ) {
    if ( curr->type() != ros_babel_fish::MessageTypes::Compound )
      return;
    const auto &c = curr->as<ros_babel_fish::CompoundMessage>();
    if ( !c.containsKey( token ) ) {
      RCLCPP_INFO_THROTTLE( logger, *clock, 9000,
                            "[BatteryMonitor] Path '%s' does not exist in message", path.c_str() );
      return;
    }
    curr = &c[token];
  }

  // 2. Handle Arrays
  if ( curr->type() == ros_babel_fish::MessageTypes::Array ) {
    auto &arr_base = curr->as<ros_babel_fish::ArrayMessageBase>();

    ros_babel_fish::invoke_for_array_message( arr_base, [&]( const auto &typed_arr ) {
      using ValType = std::decay_t<decltype( typed_arr[0] )>;
      if constexpr ( std::is_arithmetic_v<ValType> ) {
        for ( size_t i = 0; i < typed_arr.size(); ++i ) {
          check_func( static_cast<double>( typed_arr[i] ) );
        }
      } else {
        RCLCPP_INFO_THROTTLE( logger, *clock, 9000,
                              "[BatteryMonitor] Unsupported array element type in path '%s'",
                              path.c_str() );
      }
    } );
    return;
  }

  // 3. Handle Scalars
  if ( curr->type() != ros_babel_fish::MessageTypes::Compound ) {
    try {
      check_func( curr->value<double>() );
    } catch ( ... ) {
      RCLCPP_INFO_THROTTLE( logger, *clock, 9000,
                            "[BatteryMonitor] Unsupported array element type in path '%s'",
                            path.c_str() );
    }
  }
}
} // namespace

void BatteryMonitorPlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  auto ns = getPluginName();
  using Opts = hector::ParameterOptions<double>;

  low_cell_threshold_param_ = hector::createReconfigurableParameter(
      node, ns + ".low_cell_threshold", std::ref( low_cell_threshold_ ), "Low-cell threshold",
      Opts().onValidate( []( auto v ) { return v > 0.0; } ) );

  mute_duration_param_ = hector::createReconfigurableParameter(
      node, ns + ".mute_duration_sec", std::ref( mute_duration_sec_ ), "Mute duration",
      Opts().onValidate( []( auto v ) { return v > 0.0; } ) );

  ignore_nan_voltage_param_ = hector::createReconfigurableParameter(
      node, ns + ".ignore_nan_voltage", std::ref( ignore_nan_voltage_ ), "Ignore NaN cell voltages" );

  ignore_zero_voltage_param_ = hector::createReconfigurableParameter(
      node, ns + ".ignore_zero_voltage", std::ref( ignore_zero_voltage_ ),
      "Ignore zero cell voltages" );

  hector_gamepad_plugin_interface::VibrationPatternDefaults pattern_defaults;
  pattern_defaults.on_durations_sec = { 0.2, 0.2 };
  pattern_defaults.off_durations_sec = { 0.2, 5.0 };
  pattern_defaults.intensity = 0.8;
  pattern_defaults.cycle = true;
  vibration_pattern_id_ = ns + ".low_voltage";
  if ( feedback_manager_ ) {
    feedback_manager_->createVibrationPattern( vibration_pattern_id_, pattern_defaults );
    feedback_manager_->setPatternActive( vibration_pattern_id_, false );
  }

  node_->declare_parameters<std::vector<std::string>>(
      ns,
      { { "cell_voltage_fields", { "cell_voltages_battery1_mv", "cell_voltages_battery2_mv" } } } );

  cell_voltage_fields_ = node_->get_parameter( ns + ".cell_voltage_fields" ).as_string_array();

  fish_ = ros_babel_fish::BabelFish::make_shared();

  // Create a timer to check for the topic existence
  // This prevents blocking the main thread during initialization.
  subscription_timer_ =
      node_->create_wall_timer( std::chrono::seconds( 1 ), [this]() { trySubscribe(); } );
  trySubscribe();
}

void BatteryMonitorPlugin::trySubscribe()
{
  std::string topic_name = node_->get_effective_namespace();
  topic_name += "/battery";

  // Check if topic exists in the ROS graph
  const auto topics = node_->get_topic_names_and_types();
  const auto it = topics.find( topic_name );

  if ( it != topics.end() ) {
    RCLCPP_INFO( node_->get_logger(), "[BatteryMonitor] Found topic '%s'. Subscribing.",
                 topic_name.c_str() );

    // Create the subscription now that we know the topic (and implicitly the type) exists
    battery_subscription_ = fish_->create_subscription(
        *node_, topic_name, 10, [this]( const ros_babel_fish::CompoundMessage::SharedPtr msg ) {
          onBatteryMessage( msg );
        } );

    // Clean up timer
    if ( subscription_timer_ ) {
      subscription_timer_->cancel();
      subscription_timer_.reset();
    }
  } else {
    RCLCPP_DEBUG_THROTTLE( node_->get_logger(), *node_->get_clock(), 5000,
                           "[BatteryMonitor] Waiting for topic '%s'...", topic_name.c_str() );
  }
}

void BatteryMonitorPlugin::activate()
{
  active_ = true;
  low_voltage_detected_ = false;
  muted_until_ = rclcpp::Time( 0, 0, node_->get_clock()->get_clock_type() );
  if ( feedback_manager_ ) {
    feedback_manager_->setPatternActive( vibration_pattern_id_, false );
  }
}

void BatteryMonitorPlugin::deactivate()
{
  active_ = false;
  low_voltage_detected_ = false;
  if ( feedback_manager_ ) {
    feedback_manager_->setPatternActive( vibration_pattern_id_, false );
  }
}

void BatteryMonitorPlugin::handlePress( const std::string &function, const std::string &id )
{
  (void)id;
  if ( active_ && function == "mute" ) {
    const bool is_muted = isMuted();
    muted_until_ = is_muted ? rclcpp::Time( 0, 0, node_->get_clock()->get_clock_type() )
                            : node_->now() + rclcpp::Duration::from_seconds( mute_duration_sec_ );
    RCLCPP_INFO( node_->get_logger(), "[BatteryMonitor] %s vibration",
                 is_muted ? "Unmuted" : "Muted" );
  }
}

void BatteryMonitorPlugin::update()
{
  if ( !feedback_manager_ ) {
    return;
  }
  const bool should_vibrate = active_ && !isMuted() && low_voltage_detected_;
  feedback_manager_->setPatternActive( vibration_pattern_id_, should_vibrate );
}

void BatteryMonitorPlugin::onBatteryMessage( const ros_babel_fish::CompoundMessage::SharedPtr &msg )
{
  if ( !active_ || !msg )
    return;

  low_voltage_detected_ = false;
  auto check_val = [&]( const double v ) {
    if ( ignore_nan_voltage_ && std::isnan( v ) )
      return;
    if ( ignore_zero_voltage_ && v == 0.0 )
      return;
    if ( v < low_cell_threshold_ ) {
      low_voltage_detected_ = true;
    }
  };

  for ( const auto &path : cell_voltage_fields_ ) {
    checkPath( *msg, path, check_val, node_->get_logger(), node_->get_clock() );
    if ( low_voltage_detected_ )
      break;
  }
}

bool BatteryMonitorPlugin::isMuted() const
{
  return muted_until_.nanoseconds() > 0 && node_->now() < muted_until_;
}

} // namespace hector_gamepad_manager_plugins

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::BatteryMonitorPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )
