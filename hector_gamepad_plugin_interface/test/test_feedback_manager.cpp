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
  auto pattern = std::make_shared<VibrationPattern>();
  VibrationPatternDefaults defaults;
  defaults.on_durations_sec = { 0.03 };
  defaults.off_durations_sec = { 0.02 };
  defaults.intensity = 0.4;
  defaults.cycle = false;
  pattern->configure( node, "pattern.feedback", defaults );

  FeedbackManager manager;
  manager.registerVibrationPattern( "pattern", pattern );
  manager.setPatternActive( "pattern", true );

  EXPECT_TRUE( waitForCondition( [&]() { return manager.getVibrationIntensity() > 0.0; },
                                 std::chrono::milliseconds( 80 ) ) );

  EXPECT_TRUE( waitForCondition( [&]() { return manager.getVibrationIntensity() == 0.0; },
                                 std::chrono::milliseconds( 120 ) ) );

  EXPECT_DOUBLE_EQ( manager.getVibrationIntensity(), -1.0 );
}

TEST( FeedbackManager, InactivePatternsReturnIdle )
{
  auto node = makeNode( "feedback_inactive" );
  auto pattern = std::make_shared<VibrationPattern>();
  VibrationPatternDefaults defaults;
  defaults.on_durations_sec = { 0.02 };
  defaults.off_durations_sec = { 0.01 };
  defaults.intensity = 0.2;
  defaults.cycle = true;
  pattern->configure( node, "pattern.inactive", defaults );

  FeedbackManager manager;
  manager.registerVibrationPattern( "pattern", pattern );
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
