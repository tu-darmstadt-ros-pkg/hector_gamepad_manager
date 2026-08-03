#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <hector_gamepad_plugin_interface/feedback_manager.hpp>
#include <hector_gamepad_plugin_interface/vibration_pattern.hpp>

using hector_gamepad_plugin_interface::FeedbackManager;
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

TEST( FeedbackManager, EmitsSingleZeroAfterStopping )
{
  auto node = makeNode( "feedback_manager" );
  VibrationPatternDefaults defaults;
  defaults.on_durations_sec = { 0.03 };
  defaults.off_durations_sec = { 0.02 };
  defaults.intensity = 0.4;
  defaults.cycle = false;

  FeedbackManager manager;
  manager.initialize( node );
  manager.createVibrationPattern( "pattern", defaults );
  manager.setPatternActive( "pattern", true );

  EXPECT_TRUE( waitForCondition( [&]() { return manager.getVibrationIntensity() > 0.0; },
                                 std::chrono::milliseconds( 80 ) ) );

  EXPECT_TRUE( waitForCondition( [&]() { return manager.getVibrationIntensity() == 0.0; },
                                 std::chrono::milliseconds( 120 ) ) );

  EXPECT_DOUBLE_EQ( manager.getVibrationIntensity(), -1.0 );
}

// A finished one-shot pattern turns itself off and can be retriggered with a later
// setPatternActive(true). Previously the pattern stayed "active" forever after the first
// playthrough, so every later activation was a silent no-op.
TEST( FeedbackManager, OneShotPatternCanBeRetriggered )
{
  auto node = makeNode( "feedback_retrigger" );
  VibrationPatternDefaults defaults;
  defaults.on_durations_sec = { 0.03 };
  defaults.off_durations_sec = { 0.0 };
  defaults.intensity = 0.4;
  defaults.cycle = false;

  FeedbackManager manager;
  manager.initialize( node );
  manager.createVibrationPattern( "pattern", defaults );

  manager.setPatternActive( "pattern", true );
  EXPECT_TRUE( waitForCondition( [&]() { return manager.getVibrationIntensity() > 0.0; },
                                 std::chrono::milliseconds( 80 ) ) );
  // Auto-off runs inside getVibrationIntensity, mirroring the periodic publish tick.
  EXPECT_TRUE( waitForCondition(
      [&]() {
        manager.getVibrationIntensity();
        return !manager.isActive( "pattern" );
      },
      std::chrono::milliseconds( 120 ) ) );

  // Second trigger must play the pattern again.
  manager.setPatternActive( "pattern", true );
  EXPECT_TRUE( waitForCondition( [&]() { return manager.getVibrationIntensity() > 0.0; },
                                 std::chrono::milliseconds( 80 ) ) );
}

TEST( FeedbackManager, InactivePatternsReturnIdle )
{
  auto node = makeNode( "feedback_inactive" );
  VibrationPatternDefaults defaults;
  defaults.on_durations_sec = { 0.02 };
  defaults.off_durations_sec = { 0.01 };
  defaults.intensity = 0.2;
  defaults.cycle = true;

  FeedbackManager manager;
  manager.initialize( node );
  manager.createVibrationPattern( "pattern", defaults );
  manager.setPatternActive( "pattern", false );

  EXPECT_DOUBLE_EQ( manager.getVibrationIntensity(), -1.0 );
}

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  ::testing::InitGoogleTest( &argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
