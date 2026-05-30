#include <gmock/gmock.h>

#include <hector_gamepad_manager/plugin_param_loader.hpp>

#include <rclcpp/parameter.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <map>

using hector_gamepad_manager::flattenPluginParams;
using hector_gamepad_manager::loadPluginParamSet;

namespace
{
std::map<std::string, rclcpp::Parameter> toMap( const std::vector<rclcpp::Parameter> &params )
{
  std::map<std::string, rclcpp::Parameter> m;
  for ( const auto &p : params ) m.emplace( p.get_name(), p );
  return m;
}

std::string testConfigDir()
{
  return ( std::filesystem::path( __FILE__ ).parent_path() / "config" ).string();
}
} // namespace

TEST( PluginParamLoader, InfersScalarTypes )
{
  const auto params = toMap( flattenPluginParams( YAML::Load( R"(
drive_plugin:
  a_bool: true
  an_int: 3
  a_double: 1.5
  a_string: hello
)" ) ) );

  ASSERT_EQ( params.count( "drive_plugin.a_bool" ), 1u );
  EXPECT_EQ( params.at( "drive_plugin.a_bool" ).get_type(), rclcpp::ParameterType::PARAMETER_BOOL );
  EXPECT_TRUE( params.at( "drive_plugin.a_bool" ).as_bool() );

  EXPECT_EQ( params.at( "drive_plugin.an_int" ).get_type(),
             rclcpp::ParameterType::PARAMETER_INTEGER );
  EXPECT_EQ( params.at( "drive_plugin.an_int" ).as_int(), 3 );

  EXPECT_EQ( params.at( "drive_plugin.a_double" ).get_type(),
             rclcpp::ParameterType::PARAMETER_DOUBLE );
  EXPECT_DOUBLE_EQ( params.at( "drive_plugin.a_double" ).as_double(), 1.5 );

  EXPECT_EQ( params.at( "drive_plugin.a_string" ).get_type(),
             rclcpp::ParameterType::PARAMETER_STRING );
  EXPECT_EQ( params.at( "drive_plugin.a_string" ).as_string(), "hello" );
}

TEST( PluginParamLoader, IntegerVsDoubleDependsOnDecimalPoint )
{
  // Crux of the type convention: a bare integer is inferred as int, a decimal as double.
  const auto params = toMap( flattenPluginParams( YAML::Load( "p:\n  whole: 1\n  decimal: 1.0\n" ) ) );
  EXPECT_EQ( params.at( "p.whole" ).get_type(), rclcpp::ParameterType::PARAMETER_INTEGER );
  EXPECT_EQ( params.at( "p.decimal" ).get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE );
}

TEST( PluginParamLoader, FlattensNestedMapsToDotNotation )
{
  const auto params = toMap( flattenPluginParams( YAML::Load( "a:\n  b:\n    c: 5\n" ) ) );
  ASSERT_EQ( params.count( "a.b.c" ), 1u );
  EXPECT_EQ( params.at( "a.b.c" ).as_int(), 5 );
}

TEST( PluginParamLoader, InfersHomogeneousArrays )
{
  const auto params = toMap( flattenPluginParams( YAML::Load( R"(
p:
  strings: [a, b]
  doubles: [1.0, 2.0]
  empty: []
)" ) ) );
  EXPECT_EQ( params.at( "p.strings" ).get_type(), rclcpp::ParameterType::PARAMETER_STRING_ARRAY );
  EXPECT_EQ( params.at( "p.doubles" ).get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY );
  // Policy: empty sequence -> empty string array.
  EXPECT_EQ( params.at( "p.empty" ).get_type(), rclcpp::ParameterType::PARAMETER_STRING_ARRAY );
  EXPECT_TRUE( params.at( "p.empty" ).as_string_array().empty() );
}

TEST( PluginParamLoader, LoadsNamedSetFromFile )
{
  const auto params = toMap( loadPluginParamSet( testConfigDir(), "drive_a" ) );
  ASSERT_EQ( params.count( "drive_plugin.max_linear_speed" ), 1u );
  EXPECT_DOUBLE_EQ( params.at( "drive_plugin.max_linear_speed" ).as_double(), 1.0 );
}

TEST( PluginParamLoader, MissingSetReturnsEmpty )
{
  EXPECT_TRUE( loadPluginParamSet( testConfigDir(), "does_not_exist" ).empty() );
}
