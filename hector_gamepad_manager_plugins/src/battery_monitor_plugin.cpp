#include "hector_gamepad_manager_plugins/battery_monitor_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <type_traits>

#include <pluginlib/class_list_macros.hpp>
#include <ros_babel_fish/messages/array_message.hpp>
#include <ros_babel_fish/method_invoke_helpers.hpp>

namespace hector_gamepad_manager_plugins
{

namespace
{
constexpr double kMinIntensity = 0.0;
constexpr double kMaxIntensity = 1.0;

// Split a path like "foo.bar.baz" into tokens.
std::vector<std::string> splitPath( const std::string &path )
{
  std::vector<std::string> tokens;
  std::stringstream ss( path );
  std::string item;
  while ( std::getline( ss, item, '.' ) ) {
    if ( !item.empty() )
      tokens.push_back( item );
  }
  return tokens;
}

} // namespace

void BatteryMonitorPlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  const std::string plugin_namespace = getPluginName();

  low_cell_threshold_param_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".low_cell_threshold_mv", std::ref( low_cell_threshold_mv_ ),
      "Low-cell warning threshold in millivolts.",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value > 0.0; } ) );

  vibration_intensity_param_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".vibration_intensity", std::ref( vibration_intensity_ ),
      "Vibration intensity to emit on low cell voltage.",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value >= kMinIntensity && value <= kMaxIntensity; } ) );

  mute_duration_param_ = hector::createReconfigurableParameter(
      node, plugin_namespace + ".mute_duration_sec", std::ref( mute_duration_sec_ ),
      "Mute duration (seconds) after pressing the mute button.",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value > 0.0; } ) );

  node_->declare_parameters<std::vector<std::string>>(
      plugin_namespace,
      { { "cell_voltage_fields", { "cell_voltages_battery1_mv", "cell_voltages_battery2_mv" } } } );

  cell_voltage_fields_ =
      node_->get_parameter( plugin_namespace + ".cell_voltage_fields" ).as_string_array();

  fish_ = ros_babel_fish::BabelFish::make_shared();
  const std::string node_namespace = node_->get_effective_namespace();
  battery_subscription_ = fish_->create_subscription(
      *node_, node_namespace + "/battery", 10,
      [this]( const ros_babel_fish::CompoundMessage::SharedPtr msg ) { onBatteryMessage( msg ); } );

  RCLCPP_INFO( node_->get_logger(), "[BatteryMonitor] Initialized. Listening on '%s'",
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
  if ( !active_ )
    return;

  if ( function == "mute" ) {
    const auto now = node_->now();
    if ( isMuted() ) {
      muted_until_ = rclcpp::Time( 0, 0, node_->get_clock()->get_clock_type() );
      RCLCPP_INFO( node_->get_logger(), "[BatteryMonitor] Unmuted vibration feedback" );
    } else {
      muted_until_ = now + rclcpp::Duration::from_seconds( mute_duration_sec_ );
      RCLCPP_INFO( node_->get_logger(),
                   "[BatteryMonitor] Muted vibration until %.2f (sec from epoch)",
                   muted_until_.seconds() );
    }
  }
}

double BatteryMonitorPlugin::getVibrationFeedback()
{
  if ( !active_ )
    return 0.0;

  if ( isMuted() )
    return 0.0;

  if ( !low_voltage_detected_ )
    return 0.0;

  return std::clamp( vibration_intensity_, kMinIntensity, kMaxIntensity );
}

void BatteryMonitorPlugin::onBatteryMessage( const ros_babel_fish::CompoundMessage::SharedPtr &msg )
{
  if ( !active_ || !msg )
    return;

  std::vector<double> values;
  bool any_field_found = false;

  for ( const auto &path : cell_voltage_fields_ ) {
    if ( collectFieldValues( *msg, path, values ) ) {
      any_field_found = true;
    }
  }

  if ( !any_field_found )
    return;

  low_voltage_detected_ = false;
  for ( const auto value : values ) {
    if ( value > 0.0 && !std::isnan( value ) && value < low_cell_threshold_mv_ ) {
      low_voltage_detected_ = true;
      break;
    }
  }
  RCLCPP_INFO( node_->get_logger(), "[BatteryMonitor] Low voltage detected: %s",
               low_voltage_detected_ ? "true" : "false" );
}

bool BatteryMonitorPlugin::collectFieldValues( const ros_babel_fish::CompoundMessage &root,
                                               const std::string &path, std::vector<double> &out ) const
{
  const auto tokens = splitPath( path );
  if ( tokens.empty() )
    return false;

  const ros_babel_fish::Message *current = &root;

  // 1. Traverse
  for ( const auto &token : tokens ) {
    if ( current->type() != ros_babel_fish::MessageTypes::Compound ) {
      return false;
    }

    const auto &compound = current->as<ros_babel_fish::CompoundMessage>();
    if ( !compound.containsKey( token ) ) {
      return false;
    }

    current = &compound[token];
  }

  // 2. Array Case
  if ( current->type() == ros_babel_fish::MessageTypes::Array ) {
    const auto &array_msg = current->as<ros_babel_fish::ArrayMessageBase>();

    const auto extract_typed_array = [&]( auto type_tag ) {
      using T = decltype( type_tag );
      auto copy_elements = [&]( const auto &typed_array ) {
        out.reserve( out.size() + typed_array.size() );
        for ( size_t i = 0; i < typed_array.size(); ++i ) {
          out.push_back( static_cast<double>( typed_array[i] ) );
        }
      };

      if ( array_msg.isFixedSize() ) {
        copy_elements( array_msg.as<ros_babel_fish::ArrayMessage_<T, false, true>>() );
      } else if ( array_msg.isBounded() ) {
        copy_elements( array_msg.as<ros_babel_fish::ArrayMessage_<T, true, false>>() );
      } else {
        copy_elements( array_msg.as<ros_babel_fish::ArrayMessage_<T, false, false>>() );
      }
    };
    // clang-format off
    switch ( array_msg.elementType() ) {
      case ros_babel_fish::MessageTypes::Int8:   extract_typed_array( int8_t{} ); break;
      case ros_babel_fish::MessageTypes::UInt8:  extract_typed_array( uint8_t{} ); break;
      case ros_babel_fish::MessageTypes::Int16:  extract_typed_array( int16_t{} ); break;
      case ros_babel_fish::MessageTypes::UInt16: extract_typed_array( uint16_t{} ); break;
      case ros_babel_fish::MessageTypes::Int32:  extract_typed_array( int32_t{} ); break;
      case ros_babel_fish::MessageTypes::UInt32: extract_typed_array( uint32_t{} ); break;
      case ros_babel_fish::MessageTypes::Int64:  extract_typed_array( int64_t{} ); break;
      case ros_babel_fish::MessageTypes::UInt64: extract_typed_array( uint64_t{} ); break;
      case ros_babel_fish::MessageTypes::Float:  extract_typed_array( float{} ); break;
      case ros_babel_fish::MessageTypes::Double: extract_typed_array( double{} ); break;
      default: return false;
    }
    // clang-format on
    return true;
  }

  // 3. Scalar Case
  if ( current->type() != ros_babel_fish::MessageTypes::Compound ) {
    try {
      out.push_back( current->value<double>() );
      return true;
    } catch ( ... ) {
    }
  }

  return false;
}

bool BatteryMonitorPlugin::isMuted() const
{
  if ( muted_until_.nanoseconds() == 0 )
    return false;

  return node_->now() < muted_until_;
}

} // namespace hector_gamepad_manager_plugins

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::BatteryMonitorPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )