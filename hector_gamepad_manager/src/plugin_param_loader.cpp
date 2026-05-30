#include "hector_gamepad_manager/plugin_param_loader.hpp"

#include <hector_gamepad_plugin_interface/yaml_helper.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <rclcpp/logging.hpp>

namespace hector_gamepad_manager
{
namespace
{
using hector_gamepad_plugin_interface::is_bool;
using hector_gamepad_plugin_interface::is_double;
using hector_gamepad_plugin_interface::is_int;
using hector_gamepad_plugin_interface::is_string;

rclcpp::Logger logger() { return rclcpp::get_logger( "plugin_param_loader" ); }

// Convert a homogeneous scalar sequence into a typed-array rclcpp::Parameter.
// Returns false (appends nothing) for mixed-type or nested sequences.
bool appendSequence( const std::string &name, const YAML::Node &seq,
                     std::vector<rclcpp::Parameter> &out )
{
  // Policy mirrors Blackboard::set_list_from_yaml: empty sequence -> empty string array.
  if ( seq.size() == 0 ) {
    out.emplace_back( name, std::vector<std::string>{} );
    return true;
  }

  bool all_bool = true, all_int = true, all_double = true, all_string = true;
  for ( const auto &item : seq ) {
    if ( item.IsMap() || item.IsSequence() )
      return false; // nested containers are not representable as a ROS parameter
    if ( !is_bool( item ) )
      all_bool = false;
    if ( !is_int( item ) )
      all_int = false;
    if ( !is_double( item ) )
      all_double = false;
    if ( !is_string( item ) )
      all_string = false;
  }

  // Precedence bool > int > double > string (same as the scalar path and the Blackboard).
  if ( all_bool ) {
    std::vector<bool> v;
    for ( const auto &item : seq ) v.push_back( *is_bool( item ) );
    out.emplace_back( name, v );
  } else if ( all_int ) {
    std::vector<int64_t> v;
    for ( const auto &item : seq ) v.push_back( *is_int( item ) );
    out.emplace_back( name, v );
  } else if ( all_double ) {
    std::vector<double> v;
    for ( const auto &item : seq ) v.push_back( *is_double( item ) );
    out.emplace_back( name, v );
  } else if ( all_string ) {
    std::vector<std::string> v;
    for ( const auto &item : seq ) v.push_back( *is_string( item ) );
    out.emplace_back( name, v );
  } else {
    return false; // mixed scalar types
  }
  return true;
}

void appendScalar( const std::string &name, const YAML::Node &node,
                   std::vector<rclcpp::Parameter> &out )
{
  if ( auto b = is_bool( node ) ) {
    out.emplace_back( name, *b );
  } else if ( auto i = is_int( node ) ) {
    out.emplace_back( name, *i );
  } else if ( auto d = is_double( node ) ) {
    out.emplace_back( name, *d );
  } else if ( auto s = is_string( node ) ) {
    out.emplace_back( name, *s );
  }
}

void flatten( const YAML::Node &node, const std::string &prefix,
              std::vector<rclcpp::Parameter> &out )
{
  if ( node.IsMap() ) {
    for ( const auto &entry : node ) {
      const std::string key = entry.first.as<std::string>();
      const std::string child = prefix.empty() ? key : prefix + "." + key;
      flatten( entry.second, child, out );
    }
  } else if ( node.IsSequence() ) {
    if ( !prefix.empty() && !appendSequence( prefix, node, out ) ) {
      RCLCPP_WARN( logger(), "Skipping plugin param '%s': mixed-type or nested sequence",
                   prefix.c_str() );
    }
  } else if ( node.IsScalar() ) {
    if ( !prefix.empty() )
      appendScalar( prefix, node, out );
  }
}

} // namespace

std::vector<rclcpp::Parameter> flattenPluginParams( const YAML::Node &plugin_params )
{
  std::vector<rclcpp::Parameter> out;
  if ( plugin_params && plugin_params.IsMap() )
    flatten( plugin_params, "", out );
  return out;
}

std::string resolvePluginParamsPath( const std::string &config_directory, const std::string &name )
{
  std::filesystem::path dir( config_directory );
  if ( !dir.is_absolute() ) {
    dir = std::filesystem::path(
              ament_index_cpp::get_package_share_directory( "hector_gamepad_manager" ) ) /
          config_directory;
  }
  return ( dir / "plugin_params" / ( name + ".yaml" ) ).string();
}

std::vector<rclcpp::Parameter> loadPluginParamSet( const std::string &config_directory,
                                                   const std::string &name )
{
  std::string path;
  try {
    path = resolvePluginParamsPath( config_directory, name );
  } catch ( const std::exception &e ) {
    RCLCPP_WARN( logger(), "Could not resolve plugin param set '%s': %s", name.c_str(), e.what() );
    return {};
  }

  if ( !std::filesystem::exists( path ) ) {
    RCLCPP_WARN( logger(), "Plugin param set file not found: %s", path.c_str() );
    return {};
  }

  try {
    return flattenPluginParams( YAML::LoadFile( path ) );
  } catch ( const std::exception &e ) {
    RCLCPP_WARN( logger(), "Failed to load plugin param set '%s': %s", path.c_str(), e.what() );
    return {};
  }
}

} // namespace hector_gamepad_manager
