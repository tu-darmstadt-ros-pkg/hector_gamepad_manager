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

  node_->declare_parameters<std::string>( plugin_namespace,
                                          { { "battery_topic", "battery_status" } } );
  node_->declare_parameters<std::vector<std::string>>(
      plugin_namespace,
      { { "cell_voltage_fields", { "cell_voltages_battery1_mv", "cell_voltages_battery2_mv" } } } );

  const auto battery_topic = node_->get_parameter( plugin_namespace + ".battery_topic" ).as_string();
  cell_voltage_fields_ =
      node_->get_parameter( plugin_namespace + ".cell_voltage_fields" ).as_string_array();

  fish_ = ros_babel_fish::BabelFish::make_shared();
  battery_subscription_ = fish_->create_subscription(
      *node_, "/athena/battery", 10,
      [this]( const ros_babel_fish::CompoundMessage::SharedPtr msg ) { onBatteryMessage( msg ); } );
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
      RCLCPP_INFO( node_->get_logger(), "[battery_monitor_plugin] Unmuted vibration feedback" );
    } else {
      muted_until_ = now + rclcpp::Duration::from_seconds( mute_duration_sec_ );
      RCLCPP_INFO( node_->get_logger(),
                   "[battery_monitor_plugin] Muted vibration until %.2f (sec from epoch)",
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
  if ( !active_ )
    return;

  if ( !msg ) {
    RCLCPP_WARN( node_->get_logger(), "[battery_monitor_plugin] Received null battery message" );
    return;
  }

  std::vector<double> values;
  bool any_field_found = false;
  for ( const auto &path : cell_voltage_fields_ ) {
    if ( collectFieldValues( *msg, path, values ) ) {
      any_field_found = true;
    } else {
      RCLCPP_WARN_THROTTLE( node_->get_logger(), *node_->get_clock(), 5000,
                            "[battery_monitor_plugin] Field '%s' not found in battery message",
                            path.c_str() );
    }
  }

  if ( !any_field_found ) {
    low_voltage_detected_ = false;
    return;
  }

  low_voltage_detected_ = false;
  std::stringstream ss;
  for ( const auto value : values ) {
    ss << value << ", ";
    if ( value == 0.0 || std::isnan( value ) )
      continue;
    if ( value < low_cell_threshold_mv_ ) {
      low_voltage_detected_ = true;
      break;
    }
  }
  RCLCPP_INFO( node_->get_logger(),
               "[battery_monitor_plugin] Cell voltages: [%s], low voltage detected: %s",
               ss.str().c_str(), low_voltage_detected_ ? "true" : "false" );
}

bool BatteryMonitorPlugin::collectFieldValues( const ros_babel_fish::CompoundMessage &root,
                                               const std::string &path, std::vector<double> &out ) const
{
  const auto tokens = splitPath( path );
  if ( tokens.empty() ) {
    return false;
  }

  const ros_babel_fish::Message *current = &root;

  // 1. Traverse to the target field
  for ( const auto &token : tokens ) {
    if ( current->type() != ros_babel_fish::MessageTypes::Compound ) {
      return false;
    }

    const auto &compound = current->as<ros_babel_fish::CompoundMessage>();

    // Safety check: ensure field exists to avoid exception
    if ( !compound.containsKey( token ) ) {
      return false;
    }

    const ros_babel_fish::Message &member = compound[token];
    current = &member;
  }

  // 2. Handle Array Types (Fixed, Bounded, or Dynamic)
  if ( current->type() == ros_babel_fish::MessageTypes::Array ) {
    const auto &array_msg = current->as<ros_babel_fish::ArrayMessageBase>();

    // Helper to extract data from ANY array type (Fixed, Bounded, or Dynamic)
    const auto push_array_as_double = [&]( auto tag ) {
      using T = decltype( tag );

      // Inner helper to actually loop and push values
      auto copy_values = [&]( const auto &typed_array ) {
        out.reserve( out.size() + typed_array.size() );
        for ( size_t i = 0; i < typed_array.size(); ++i ) {
          out.push_back( static_cast<double>( typed_array[i] ) );
        }
      };

      // Check which specific array class implements this message
      if ( array_msg.isFixedSize() ) {
        // Cast to Fixed Size Array: ArrayMessage_<T, Bounded=false, Fixed=true>
        copy_values( array_msg.as<ros_babel_fish::ArrayMessage_<T, false, true>>() );
      } else if ( array_msg.isBounded() ) {
        // Cast to Bounded Array: ArrayMessage_<T, Bounded=true, Fixed=false>
        copy_values( array_msg.as<ros_babel_fish::ArrayMessage_<T, true, false>>() );
      } else {
        // Cast to Dynamic Array: ArrayMessage_<T, Bounded=false, Fixed=false>
        copy_values( array_msg.as<ros_babel_fish::ArrayMessage_<T, false, false>>() );
      }
    };

    switch ( array_msg.elementType() ) {
    case ros_babel_fish::MessageTypes::Int8:
      push_array_as_double( int8_t{} );
      break;
    case ros_babel_fish::MessageTypes::UInt8:
      push_array_as_double( uint8_t{} );
      break;
    case ros_babel_fish::MessageTypes::Int16:
      push_array_as_double( int16_t{} );
      break;
    case ros_babel_fish::MessageTypes::UInt16:
      push_array_as_double( uint16_t{} );
      break;
    case ros_babel_fish::MessageTypes::Int32:
      push_array_as_double( int32_t{} );
      break;
    case ros_babel_fish::MessageTypes::UInt32:
      push_array_as_double( uint32_t{} );
      break;
    case ros_babel_fish::MessageTypes::Int64:
      push_array_as_double( int64_t{} );
      break;
    case ros_babel_fish::MessageTypes::UInt64:
      push_array_as_double( uint64_t{} );
      break;
    case ros_babel_fish::MessageTypes::Float:
      push_array_as_double( float{} );
      break;
    case ros_babel_fish::MessageTypes::Double:
      push_array_as_double( double{} );
      break;
    default:
      RCLCPP_WARN( node_->get_logger(), "Field '%s' is an array of non-numeric type", path.c_str() );
      return false;
    }
    return true;
  }

  // 3. Handle Scalar Case (Single number)
  if ( current->type() != ros_babel_fish::MessageTypes::Compound ) {
    try {
      out.push_back( current->value<double>() );
      return true;
    } catch ( ... ) {
      // Ignore non-numeric scalars (strings, etc.)
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
