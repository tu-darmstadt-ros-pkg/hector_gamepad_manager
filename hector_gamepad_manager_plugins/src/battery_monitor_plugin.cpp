#include "hector_gamepad_manager_plugins/battery_monitor_plugin.hpp"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

namespace hector_gamepad_manager_plugins
{

namespace
{
constexpr double kMinIntensity = 0.0;
constexpr double kMaxIntensity = 1.0;
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

  battery_subscription_ =
      node_->create_subscription<athena_firmware_interface_msgs::msg::BatteryStatus>(
          "battery", rclcpp::QoS( rclcpp::KeepLast( 5 ) ),
          [this]( const athena_firmware_interface_msgs::msg::BatteryStatus &msg ) {
            onBatteryStatus( msg );
          } );
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

void BatteryMonitorPlugin::onBatteryStatus( const athena_firmware_interface_msgs::msg::BatteryStatus &msg )
{
  if ( !active_ )
    return;

  low_voltage_detected_ =
      hasLowCell( msg.cell_voltages_battery1_mv ) || hasLowCell( msg.cell_voltages_battery2_mv );
}

bool BatteryMonitorPlugin::hasLowCell( const std::array<uint16_t, 8> &cells ) const
{
  for ( const auto cell_mv : cells ) {
    if ( cell_mv == 0 )
      continue;

    const double voltage = static_cast<double>( cell_mv );
    if ( std::isnan( voltage ) )
      continue;

    if ( voltage < low_cell_threshold_mv_ ) {
      return true;
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
