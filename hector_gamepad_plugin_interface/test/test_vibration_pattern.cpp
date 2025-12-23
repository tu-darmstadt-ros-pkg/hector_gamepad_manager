#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <hector_gamepad_plugin_interface/vibration_pattern.hpp>

using hector_gamepad_plugin_interface::VibrationPattern;
using hector_gamepad_plugin_interface::VibrationPatternDefaults;

namespace
{
std::shared_ptr<rclcpp::Node> makeNode( const std::string &name_prefix )
{
  static int counter = 0;
  return std::make_shared<rclcpp::Node>( name_prefix + "_" + std::to_string( counter++ ) );
}

bool waitForCondition( const std::function<bool()> &condition,
                       const std::chrono::milliseconds timeout )
{
  const auto start = std::chrono::steady_clock::now();
  while ( std::chrono::steady_clock::now() - start < timeout ) {
    if ( condition() ) {
      return true;
    }
    std::this_thread::sleep_for( std::chrono::milliseconds( 2 ) );
  }
  return false;
}
} // namespace

TEST( VibrationPattern, InactiveOrUnconfiguredReturnsZero )
{
  VibrationPattern pattern;
  EXPECT_DOUBLE_EQ( pattern.getIntensityNow(), 0.0 );

  auto node = makeNode( "vibration_unconfigured" );
  pattern.configure( node, "pattern.inactive" );
  pattern.setActive( false );
  EXPECT_DOUBLE_EQ( pattern.getIntensityNow(), 0.0 );
}

TEST( VibrationPattern, SinglePulseNonCyclingStopsAfterDuration )
{
  auto node = makeNode( "vibration_single_pulse" );
  VibrationPatternDefaults defaults;
  defaults.on_durations_sec = { 0.05 };
  defaults.off_durations_sec = { 0.02 };
  defaults.intensity = 0.7;
  defaults.cycle = false;

  VibrationPattern pattern;
  pattern.configure( node, "pattern.single_pulse", defaults );
  pattern.setActive( true );

  EXPECT_TRUE( waitForCondition( [&]() { return pattern.getIntensityNow() > 0.1; },
                                 std::chrono::milliseconds( 80 ) ) );

  EXPECT_TRUE( waitForCondition( [&]() { return pattern.getIntensityNow() <= 0.0; },
                                 std::chrono::milliseconds( 120 ) ) );
}

TEST( VibrationPattern, CyclingPatternRepeats )
{
  auto node = makeNode( "vibration_cycle" );
  VibrationPatternDefaults defaults;
  defaults.on_durations_sec = { 0.02 };
  defaults.off_durations_sec = { 0.02 };
  defaults.intensity = 0.6;
  defaults.cycle = true;

  VibrationPattern pattern;
  pattern.configure( node, "pattern.cycle", defaults );
  pattern.setActive( true );

  EXPECT_TRUE( waitForCondition( [&]() { return pattern.getIntensityNow() > 0.1; },
                                 std::chrono::milliseconds( 80 ) ) );

  EXPECT_TRUE( waitForCondition( [&]() { return pattern.getIntensityNow() <= 0.0; },
                                 std::chrono::milliseconds( 80 ) ) );

  EXPECT_TRUE( waitForCondition( [&]() { return pattern.getIntensityNow() > 0.1; },
                                 std::chrono::milliseconds( 120 ) ) );
}

TEST( VibrationPattern, ReactivatingResetsTiming )
{
  auto node = makeNode( "vibration_reactivate" );
  VibrationPatternDefaults defaults;
  defaults.on_durations_sec = { 0.03 };
  defaults.off_durations_sec = { 0.02 };
  defaults.intensity = 0.9;
  defaults.cycle = false;

  VibrationPattern pattern;
  pattern.configure( node, "pattern.reactivate", defaults );
  pattern.setActive( true );

  EXPECT_TRUE( waitForCondition( [&]() { return pattern.getIntensityNow() <= 0.0; },
                                 std::chrono::milliseconds( 120 ) ) );

  pattern.setActive( false );
  pattern.setActive( true );

  EXPECT_TRUE( waitForCondition( [&]() { return pattern.getIntensityNow() > 0.1; },
                                 std::chrono::milliseconds( 80 ) ) );
}

TEST( VibrationPattern, RejectsInvalidParameterUpdates )
{
  auto node = makeNode( "vibration_params" );
  VibrationPatternDefaults defaults;
  defaults.on_durations_sec = { 0.01 };
  defaults.off_durations_sec = { 0.01 };
  defaults.intensity = 0.5;
  defaults.cycle = true;

  VibrationPattern pattern;
  pattern.configure( node, "pattern.params", defaults );

  auto results = node->set_parameters( { rclcpp::Parameter( "pattern.params.intensity", 2.0 ) } );
  ASSERT_EQ( results.size(), 1u );
  EXPECT_FALSE( results[0].successful );
}

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  ::testing::InitGoogleTest( &argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
