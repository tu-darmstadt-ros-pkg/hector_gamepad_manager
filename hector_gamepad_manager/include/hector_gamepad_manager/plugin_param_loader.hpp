#ifndef HECTOR_GAMEPAD_MANAGER_PLUGIN_PARAM_LOADER_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGIN_PARAM_LOADER_HPP

#include <rclcpp/parameter.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace hector_gamepad_manager
{

/**
 * @brief Flatten a plugin-param YAML map into dot-notation rclcpp::Parameters.
 *
 * Top-level keys are plugin namespaces (snake_case plugin names); nested maps are joined with
 * '.' so the result matches how plugins declare their parameters (e.g.
 * `drive_plugin.max_linear_speed`). Scalar types are inferred with precedence
 * bool > int > double > string; homogeneous sequences become typed arrays (an empty sequence
 * becomes an empty string array). Mixed-type or nested sequences are skipped with a warning.
 *
 * @note Parameters declared as `double` MUST be written with a decimal point in the YAML
 *       (e.g. `1.0`, not `1`); otherwise they are inferred as int and the resulting override
 *       type will not match the parameter's declared type.
 */
std::vector<rclcpp::Parameter> flattenPluginParams( const YAML::Node &plugin_params );

/**
 * @brief Resolve the path of a named plugin-param set: `<config_directory>/plugin_params/<name>.yaml`.
 *
 * Absolute @p config_directory is used as-is; a relative one is resolved against the
 * `hector_gamepad_manager` package share directory.
 */
std::string resolvePluginParamsPath( const std::string &config_directory, const std::string &name );

/**
 * @brief Load and flatten the named plugin-param set into rclcpp::Parameters.
 *
 * Returns an empty vector (and logs a warning) if the file is missing or cannot be parsed.
 */
std::vector<rclcpp::Parameter> loadPluginParamSet( const std::string &config_directory,
                                                   const std::string &name );

} // namespace hector_gamepad_manager

#endif // HECTOR_GAMEPAD_MANAGER_PLUGIN_PARAM_LOADER_HPP
