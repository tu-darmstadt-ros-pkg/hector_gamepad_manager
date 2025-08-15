#include <optional>
#include <yaml-cpp/yaml.h>

#ifndef HECTOR_GAMEPAD_PLUGIN_INTERFACE_YAML_HELPER_HPP
  #define HECTOR_GAMEPAD_PLUGIN_INTERFACE_YAML_HELPER_HPP
namespace hector_gamepad_plugin_interface
{

inline std::optional<bool> is_bool( const YAML::Node &node )
{
  bool bool_value{};
  if ( YAML::convert<bool>::decode( node, bool_value ) ) {
    return bool_value;
  }
  return std::nullopt;
}
inline std::optional<int64_t> is_int( const YAML::Node &node )
{
  int64_t int_value{};
  if ( YAML::convert<int64_t>::decode( node, int_value ) ) {
    return int_value;
  }
  return std::nullopt;
}
// Note: will return true for both integer and floating-point values!
// Make sure to first check for integer if you want to distinguish them.
inline std::optional<double> is_double( const YAML::Node &node )
{
  double double_value{};
  if ( YAML::convert<double>::decode( node, double_value ) ) {
    return double_value;
  }
  return std::nullopt;
}
// Note: will return a strung for int, double, and string values.
inline std::optional<std::string> is_string( const YAML::Node &node )
{
  std::string string_value{};
  if ( YAML::convert<std::string>::decode( node, string_value ) ) {
    return string_value;
  }
  return std::nullopt;
}
} // namespace hector_gamepad_plugin_interface
#endif // HECTOR_GAMEPAD_PLUGIN_INTERFACE_YAML_HELPER_HPP