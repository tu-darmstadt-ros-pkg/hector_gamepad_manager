#include "hector_gamepad_manager_plugins/battery_monitor_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <ros_babel_fish/messages/array_message.hpp>
#include <ros_babel_fish/method_invoke_helpers.hpp>
#include <sstream>

namespace hector_gamepad_manager_plugins
{

namespace
{
// Helper: Traverses the message path and calls 'visitor' on every numeric value found.
// Handles Scalars and Arrays (Fixed, Bounded, Dynamic) transparently.
template<typename Visitor>
void checkPath( const ros_babel_fish::CompoundMessage &root, const std::string &path, Visitor visitor )
{
  const ros_babel_fish::Message *curr = &root;
  std::stringstream ss( path );
  std::string token;

  // 1. Traverse Path
  while ( std::getline( ss, token, '.' ) ) {
    if ( curr->type() != ros_babel_fish::MessageTypes::Compound )
      return;
    const auto &c = curr->as<ros_babel_fish::CompoundMessage>();
    if ( !c.containsKey( token ) )
      return;
    curr = &c[token];
  }

  // 2. Handle Arrays
  if ( curr->type() == ros_babel_fish::MessageTypes::Array ) {
    const auto &arr = curr->as<ros_babel_fish::ArrayMessageBase>();

    // Generic lambda to cast to specific array type and iterate
    auto process = [&]( auto type_tag ) {
      using T = decltype( type_tag );
      auto iterate = [&]( const auto &typed_arr ) {
        for ( size_t i = 0; i < typed_arr.size(); ++i )
          visitor( static_cast<double>( typed_arr[i] ) );
      };

      if ( arr.isFixedSize() )
        iterate( arr.as<ros_babel_fish::ArrayMessage_<T, false, true>>() );
      else if ( arr.isBounded() )
        iterate( arr.as<ros_babel_fish::ArrayMessage_<T, true, false>>() );
      else
        iterate( arr.as<ros_babel_fish::ArrayMessage_<T, false, false>>() );
    };
    // clang-format off
    using namespace ros_babel_fish;
    switch ( arr.elementType() ) {
      case MessageTypes::Int8:   process( int8_t{} ); break;
      case MessageTypes::UInt8:  process( uint8_t{} ); break;
      case MessageTypes::Int16:  process( int16_t{} ); break;
      case MessageTypes::UInt16: process( uint16_t{} ); break;
      case MessageTypes::Int32:  process( int32_t{} ); break;
      case MessageTypes::UInt32: process( uint32_t{} ); break;
      case MessageTypes::Int64:  process( int64_t{} ); break;
      case MessageTypes::UInt64: process( uint64_t{} ); break;
      case MessageTypes::Float:  process( float{} ); break;
      case MessageTypes::Double: process( double{} ); break;
      default: break; // Ignore non-numeric arrays
    }
    // clang-format on
    return;
  }

  // 3. Handle Scalars
  if ( curr->type() != ros_babel_fish::MessageTypes::Compound ) {
    try {
      visitor( curr->value<double>() );
    } catch ( ... ) {
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
      node, ns + ".low_cell_threshold_mv", std::ref( low_cell_threshold_mv_ ), "Low-cell threshold",
      Opts().onValidate( []( auto v ) { return v > 0.0; } ) );

  vibration_intensity_param_ = hector::createReconfigurableParameter(
      node, ns + ".vibration_intensity", std::ref( vibration_intensity_ ), "Vibration intensity",
      Opts().onValidate( []( auto v ) { return v >= 0.0 && v <= 1.0; } ) );

  mute_duration_param_ = hector::createReconfigurableParameter(
      node, ns + ".mute_duration_sec", std::ref( mute_duration_sec_ ), "Mute duration",
      Opts().onValidate( []( auto v ) { return v > 0.0; } ) );

  node_->declare_parameters<std::vector<std::string>>(
      ns,
      { { "cell_voltage_fields", { "cell_voltages_battery1_mv", "cell_voltages_battery2_mv" } } } );

  cell_voltage_fields_ = node_->get_parameter( ns + ".cell_voltage_fields" ).as_string_array();

  fish_ = ros_babel_fish::BabelFish::make_shared();
  battery_subscription_ = fish_->create_subscription(
      *node_, node_->get_effective_namespace() + "/battery", 10,
      [this]( const ros_babel_fish::CompoundMessage::SharedPtr msg ) { onBatteryMessage( msg ); } );

  RCLCPP_INFO( node_->get_logger(), "[BatteryMonitor] Listening on '%s'",
               battery_subscription_->get_topic_name() );
}

void BatteryMonitorPlugin::activate()
{
  active_ = true;
  low_voltage_detected_ = false;
  muted_until_ = rclcpp::Time( 0, 0, node_->get_clock()->get_clock_type() );
}

void BatteryMonitorPlugin::deactivate()
{
  active_ = false;
  low_voltage_detected_ = false;
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

double BatteryMonitorPlugin::getVibrationFeedback()
{
  return ( active_ && !isMuted() && low_voltage_detected_ )
             ? std::clamp( vibration_intensity_, 0.0, 1.0 )
             : 0.0;
}

void BatteryMonitorPlugin::onBatteryMessage( const ros_babel_fish::CompoundMessage::SharedPtr &msg )
{
  if ( !active_ || !msg )
    return;

  low_voltage_detected_ = false;
  auto check_val = [&]( double v ) {
    if ( !low_voltage_detected_ && v > 0.0 && !std::isnan( v ) && v < low_cell_threshold_mv_ ) {
      low_voltage_detected_ = true;
    }
  };

  for ( const auto &path : cell_voltage_fields_ ) {
    checkPath( *msg, path, check_val );
    if ( low_voltage_detected_ )
      break; // Stop early if low cell found
  }
  RCLCPP_INFO( node_->get_logger(), "[BatteryMonitor] Low voltage detected: %s",
               low_voltage_detected_ ? "TRUE" : "FALSE" );
}

bool BatteryMonitorPlugin::isMuted() const
{
  return muted_until_.nanoseconds() > 0 && node_->now() < muted_until_;
}

} // namespace hector_gamepad_manager_plugins

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::BatteryMonitorPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )