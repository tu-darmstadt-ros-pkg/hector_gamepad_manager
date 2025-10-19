#include "hector_gamepad_manager_plugins/blackboard_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <utility>

namespace hector_gamepad_manager_plugins
{

using hector_gamepad_plugin_interface::Blackboard;

void BlackboardPlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
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

void BlackboardPlugin::handlePress( const std::string &function, const std::string &id )
{
  if ( !active_ )
    return;

  if ( function == "toggle" ) {
    const std::string &name = getConfigValueOr<std::string>( id, "name" );
    const std::string &topic = getConfigValueOr<std::string>( id, "topic", "" );
    const bool default_value = getConfigValueOr<bool>( id, "default", false );
    onToggle( name, topic, default_value );
    return;
  } else if ( function == "hold" ) {
    const std::string &name = getConfigValueOr<std::string>( id, "name" );
    onHoldPress( name );
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
    onHoldRelease( name );
    return;
  }
  // toggle/* and set/* are no-ops on release
}

void BlackboardPlugin::onToggle( const std::string &var, const std::string &topic, bool default_value )
{
  // Fetch or create bool with default false, then invert
  bool &ref = blackboard_->get_or_emplace<bool>( var, default_value );
  ref = !ref;
  if ( !topic.empty() ) {
    if ( toggle_publisher_.find( topic ) == toggle_publisher_.end() ) {
      const auto qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ).transient_local().reliable();
      toggle_publisher_[topic] = node_->create_publisher<std_msgs::msg::Bool>( topic, qos );
    }
    auto msg = std_msgs::msg::Bool();
    msg.data = ref;
    toggle_publisher_[topic]->publish( msg );
  }
  RCLCPP_DEBUG( node_->get_logger(), "[blackboard_plugin] toggle %s -> %s", var.c_str(),
                ref ? "true" : "false" );
}

void BlackboardPlugin::onHoldPress( const std::string &var ) const
{
  bool &ref = blackboard_->get_or_emplace<bool>( var, false );
  ref = true;
  RCLCPP_DEBUG( node_->get_logger(), "[blackboard_plugin] hold %s -> true", var.c_str() );
}

void BlackboardPlugin::onHoldRelease( const std::string &var ) const
{
  bool &ref = blackboard_->get_or_emplace<bool>( var, false );
  ref = false;
  RCLCPP_DEBUG( node_->get_logger(), "[blackboard_plugin] hold %s -> false", var.c_str() );
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
