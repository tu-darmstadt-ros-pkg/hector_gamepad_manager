#include "hector_gamepad_manager_plugins/blackboard_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <utility>

namespace hector_gamepad_manager_plugins
{

using hector_gamepad_plugin_interface::Blackboard;

void BlackboardPlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  // Nothing to declare; we act purely on blackboard keys.
  RCLCPP_DEBUG( node_->get_logger(), "[blackboard_plugin] initialized" );
}

void BlackboardPlugin::activate()
{
  active_ = true;
  RCLCPP_DEBUG( node_->get_logger(), "[blackboard_plugin] activated" );
}

void BlackboardPlugin::deactivate()
{
  active_ = false;
  RCLCPP_DEBUG( node_->get_logger(), "[blackboard_plugin] deactivated" );
}

void BlackboardPlugin::bindButton( const std::string &function, const std::string &id )
{
  if ( function != "toggle" )
    return;
  const std::string topic = getConfigValueOr<std::string>( id, "topic", "" );
  // The initial value is published right away, so a forgotten one would release an e-stop.
  if ( !topic.empty() && !hasConfigValue<bool>( id, "initial" ) ) {
    throw std::invalid_argument( "Toggle '" + id + "' publishes on '" + topic +
                                 "' and must set 'initial' to true or false" );
  }
  const std::string &name = getConfigValueOr<std::string>( id, "name" );
  const bool initial_value = getConfigValueOr<bool>( id, "initial", false );
  const bool value = blackboard_->get_or_emplace<bool>( name, initial_value );
  publishToggle( topic, getConfigValueOr<std::string>( id, "ocs_topic", "" ), value );
}

void BlackboardPlugin::handlePress( const std::string &function, const std::string &id )
{
  if ( !active_ )
    return;

  if ( function == "toggle" ) {
    const std::string &name = getConfigValueOr<std::string>( id, "name" );
    const std::string &topic = getConfigValueOr<std::string>( id, "topic", "" );
    const std::string &ocs_topic = getConfigValueOr<std::string>( id, "ocs_topic", "" );
    const bool initial_value = getConfigValueOr<bool>( id, "initial", false );
    onToggle( name, topic, ocs_topic, initial_value );
    return;
  } else if ( function == "hold" ) {
    const std::string &name = getConfigValueOr<std::string>( id, "name" );
    const std::string &topic = getConfigValueOr<std::string>( id, "topic", "" );
    onHold( name, topic, true );
    return;
  } else if ( function == "set" ) {
    const std::string &to = getConfigValueOr<std::string>( id, "set" );
    const std::string &name = getConfigValueOr<std::string>( id, "name" );
    onSetString( name, to );
    return;
  }

  RCLCPP_WARN( node_->get_logger(), "[blackboard_plugin] unsupported function on press: '%s'",
               function.c_str() );
}

void BlackboardPlugin::handleRelease( const std::string &function, const std::string &id )
{
  if ( !active_ )
    return;

  // Only "hold/<var>" reacts on release → set false
  if ( function == "hold" ) {
    const auto name = getConfigValueOr<std::string>( id, "name" );
    const auto topic = getConfigValueOr<std::string>( id, "topic", "" );
    onHold( name, topic, false );
    return;
  }
  // toggle/* and set/* are no-ops on release
}

rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
BlackboardPlugin::getOrCreatePublisher( const std::string &topic )
{
  if ( publishers_.find( topic ) == publishers_.end() ) {
    const auto qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ).transient_local().reliable();
    publishers_[topic] = node_->create_publisher<std_msgs::msg::Bool>( topic, qos );
  }
  return publishers_[topic];
}

void BlackboardPlugin::onToggle( const std::string &var, const std::string &topic,
                                 const std::string &ocs_topic, bool initial_value )
{
  // Fetch or create bool with default false, then invert
  bool &ref = blackboard_->get_or_emplace<bool>( var, initial_value );
  ref = !ref;
  publishToggle( topic, ocs_topic, ref );

  RCLCPP_DEBUG( node_->get_logger(), "[blackboard_plugin] toggle %s -> %s", var.c_str(),
                ref ? "true" : "false" );
}

void BlackboardPlugin::publishToggle( const std::string &topic, const std::string &ocs_topic,
                                      bool value )
{
  const auto msg = std_msgs::msg::Bool().set__data( value );
  if ( !topic.empty() ) {
    getOrCreatePublisher( topic )->publish( msg );
  }
  // An optional second, OCS-facing topic
  if ( !ocs_topic.empty() && ocs_topic != topic ) {
    getOrCreatePublisher( ocs_topic )->publish( msg );
  }
}

void BlackboardPlugin::onHold( const std::string &var, const std::string &topic, bool value )
{
  blackboard_->get_or_emplace<bool>( var, false ) = value;
  if ( !topic.empty() ) {
    getOrCreatePublisher( topic )->publish( std_msgs::msg::Bool().set__data( value ) );
  }
  RCLCPP_DEBUG( node_->get_logger(), "[blackboard_plugin] hold %s -> %s", var.c_str(),
                value ? "true" : "false" );
}

void BlackboardPlugin::onSetString( const std::string &var, const std::string &value ) const
{
  auto &ref = blackboard_->get_or_emplace<std::string>( var, std::string{} );
  ref = value;
  RCLCPP_DEBUG( node_->get_logger(), "[blackboard_plugin] set %s -> '%s'", var.c_str(),
                value.c_str() );
}

} // namespace hector_gamepad_manager_plugins

// Export as a pluginlib class
PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::BlackboardPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )
