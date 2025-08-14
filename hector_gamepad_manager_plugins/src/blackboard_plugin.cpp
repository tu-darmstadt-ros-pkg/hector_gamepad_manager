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
  RCLCPP_INFO( node_->get_logger(), "[blackboard_plugin] initialized" );
}

std::string BlackboardPlugin::getPluginName()
{
  // Plugin name used for parameter namespaces if needed in the future
  return "blackboard_plugin";
}

void BlackboardPlugin::activate()
{
  active_ = true;
  RCLCPP_INFO( node_->get_logger(), "[blackboard_plugin] activated" );
}

void BlackboardPlugin::deactivate()
{
  active_ = false;
  RCLCPP_INFO( node_->get_logger(), "[blackboard_plugin] deactivated" );
}

bool BlackboardPlugin::startsWith( const std::string &s, const char *prefix )
{
  const auto n = std::char_traits<char>::length( prefix );
  return s.size() >= n && std::equal( prefix, prefix + n, s.begin() );
}

std::vector<std::string> BlackboardPlugin::split( const std::string &s, char delim )
{
  std::vector<std::string> out;
  std::string cur;
  for ( char c : s ) {
    if ( c == delim ) {
      out.emplace_back( std::move( cur ) );
      cur.clear();
    } else {
      cur.push_back( c );
    }
  }
  out.emplace_back( std::move( cur ) );
  return out;
}

void BlackboardPlugin::handlePress( const std::string &function )
{
  if ( !active_ )
    return;

  // Supported patterns:
  //  "toggle/<var>"
  //  "hold/<var>"
  //  "set/<var>/to/<value>"
  if ( startsWith( function, "toggle/" ) ) {
    const std::string var = function.substr( std::strlen( "toggle/" ) );
    if ( var.empty() ) {
      RCLCPP_WARN( node_->get_logger(), "[blackboard_plugin] toggle: empty variable name" );
      return;
    }
    onToggle( var );
    return;
  }

  if ( startsWith( function, "hold/" ) ) {
    const std::string var = function.substr( std::strlen( "hold/" ) );
    if ( var.empty() ) {
      RCLCPP_WARN( node_->get_logger(), "[blackboard_plugin] hold: empty variable name" );
      return;
    }
    onHoldPress( var );
    return;
  }

  if ( startsWith( function, "set/" ) ) {
    // Expect exactly: set/<var>/to/<value>
    // We allow <value> to contain further '/' (then the split size > 4). Join back everything after "to".
    const auto parts = split( function, '/' ); // e.g. ["set","my_var","to","hello_world"]
    if ( parts.size() >= 4 && parts[0] == "set" && parts[2] == "to" ) {
      const std::string &var = parts[1];
      if ( var.empty() ) {
        RCLCPP_WARN( node_->get_logger(), "[blackboard_plugin] set: empty variable name" );
        return;
      }
      // Re-join everything after index 2 as value to allow slashes
      std::string value = parts[3];
      for ( size_t i = 4; i < parts.size(); ++i ) {
        value.push_back( '/' );
        value += parts[i];
      }
      onSetString( var, value );
      return;
    }
    RCLCPP_WARN( node_->get_logger(),
                 "[blackboard_plugin] set: expected 'set/<var>/to/<value>', got '%s'",
                 function.c_str() );
    return;
  }

  RCLCPP_WARN( node_->get_logger(), "[blackboard_plugin] unsupported function on press: '%s'",
               function.c_str() );
}

void BlackboardPlugin::handleRelease( const std::string &function )
{
  if ( !active_ )
    return;

  // Only "hold/<var>" reacts on release → set false
  if ( startsWith( function, "hold/" ) ) {
    const std::string var = function.substr( std::strlen( "hold/" ) );
    if ( var.empty() ) {
      RCLCPP_WARN( node_->get_logger(), "[blackboard_plugin] hold (release): empty variable name" );
      return;
    }
    onHoldRelease( var );
    return;
  }
  // toggle/* and set/* are no-ops on release
}

void BlackboardPlugin::onToggle( const std::string &var ) const
{
  // Fetch or create bool with default false, then invert
  bool &ref = blackboard_->get_or_emplace<bool>( var, false );
  ref = !ref;
  RCLCPP_INFO( node_->get_logger(), "[blackboard_plugin] toggle %s -> %s", var.c_str(),
               ref ? "true" : "false" );
}

void BlackboardPlugin::onHoldPress( const std::string &var ) const
{
  bool &ref = blackboard_->get_or_emplace<bool>( var, false );
  ref = true;
  RCLCPP_INFO( node_->get_logger(), "[blackboard_plugin] hold %s -> true", var.c_str() );
}

void BlackboardPlugin::onHoldRelease( const std::string &var ) const
{
  bool &ref = blackboard_->get_or_emplace<bool>( var, false );
  ref = false;
  RCLCPP_INFO( node_->get_logger(), "[blackboard_plugin] hold %s -> false", var.c_str() );
}

void BlackboardPlugin::onSetString( const std::string &var, const std::string &value ) const
{
  auto &ref = blackboard_->get_or_emplace<std::string>( var, std::string{} );
  ref = value;
  RCLCPP_INFO( node_->get_logger(), "[blackboard_plugin] set %s -> '%s'", var.c_str(), value.c_str() );
}

} // namespace hector_gamepad_manager_plugins

// Export as a pluginlib class
PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::BlackboardPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )
