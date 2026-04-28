#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>
#include <rtest/publisher_mock.hpp>
#include <rtest/subscription_mock.hpp>
#include <rtest/test_clock.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include <hector_gamepad_manager/hector_gamepad_manager.hpp>

#include <chrono>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Field;
using ::testing::HasSubstr;

constexpr int MAX_BUTTONS = 25;
constexpr int MAX_AXES = 8;

class HectorGamepadManagerDoublePressTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions opts_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<hector_gamepad_manager::HectorGamepadManager> manager_;
  std::unique_ptr<rtest::TestClock> clock_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> sub_joy_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_config_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_press_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_hold_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_release_;

  sensor_msgs::msg::Joy joy_msg_;

  static std::string paramsFilePath()
  {
    auto path = std::filesystem::path( __FILE__ ).parent_path() / "config" /
                "manager_internal_double_press_params.yaml";
    return path.string();
  }

  void SetUp() override
  {
    opts_ = rclcpp::NodeOptions();
    opts_.arguments( { "--ros-args", "--params-file", paramsFilePath() } );

    node_ = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_double_press_test", opts_ );
    clock_ = std::make_unique<rtest::TestClock>( node_ );
    manager_ = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node_ );

    sub_joy_ = rtest::findSubscription<sensor_msgs::msg::Joy>( node_, "/ocs/joy" );
    pub_config_ = rtest::findPublisher<std_msgs::msg::String>( node_, "/ocs/joy_teleop_profile" );
    pub_probe_press_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/press" );
    pub_probe_hold_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/hold" );
    pub_probe_release_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/release" );

    ASSERT_TRUE( sub_joy_ );
    ASSERT_TRUE( pub_config_ );
    ASSERT_TRUE( pub_probe_press_ );
    ASSERT_TRUE( pub_probe_hold_ );
    ASSERT_TRUE( pub_probe_release_ );

    EXPECT_CALL( *pub_config_, publish( _ ) ).Times( AnyNumber() );

    resetJoy();
  }

  void resetJoy()
  {
    joy_msg_.axes = std::vector<float>( MAX_AXES, 0.0f );
    joy_msg_.buttons = std::vector<int>( MAX_BUTTONS, 0 );
    joy_msg_.axes[2] = 1.0f;
    joy_msg_.axes[5] = 1.0f;
  }

  void setButton( int id, int value ) { joy_msg_.buttons[id] = value; }

  void sendJoy() { sub_joy_->handle_message( joy_msg_ ); }

  void advanceMs( int64_t ms ) { clock_->advanceMs( ms ); }
};

// A quick tap on a double-press button must flush as press+release after the window expires.
TEST_F( HectorGamepadManagerDoublePressTest, QuickTapWithDoublePressDispatchesPressAndRelease )
{
  // No press/release/hold should fire while we are still inside the window.
  EXPECT_CALL( *pub_probe_press_, publish( _ ) ).Times( 0 );
  EXPECT_CALL( *pub_probe_release_, publish( _ ) ).Times( 0 );
  EXPECT_CALL( *pub_probe_hold_, publish( _ ) ).Times( 0 );

  setButton( 0, 1 );
  sendJoy();
  setButton( 0, 0 );
  sendJoy();

  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_release_.get() );
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_hold_.get() );

  // Advance past the window — the buffered press should flush, paired with an immediate release.
  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_release_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "release:probe:" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_hold_, publish( _ ) ).Times( 0 );

  advanceMs( 300 );
  sendJoy();
}

// Test B — Two quick presses inside the window dispatch on_double_press only.
TEST_F( HectorGamepadManagerDoublePressTest, DoublePressDispatchesDoublePressFunction )
{
  EXPECT_CALL( *pub_probe_press_, publish( _ ) ).Times( 0 );
  setButton( 0, 1 );
  sendJoy();
  setButton( 0, 0 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  advanceMs( 100 ); // still inside window

  // Second press within the window — on_double_press fires, on_press does NOT.
  EXPECT_CALL( *pub_probe_press_, publish( Field( &std_msgs::msg::String::data,
                                                  HasSubstr( "press:probe_double:" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 0 );
  setButton( 0, 1 );
  sendJoy();
}

// Two presses separated by a timeout each fire as single presses, never as double-press.
TEST_F( HectorGamepadManagerDoublePressTest, TwoPressesSeparatedByTimeoutAreBothSinglePresses )
{
  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 2 );
  EXPECT_CALL( *pub_probe_press_, publish( Field( &std_msgs::msg::String::data,
                                                  HasSubstr( "press:probe_double:" ) ) ) )
      .Times( 0 );

  // First press, release, wait past window — flushes as single press.
  setButton( 0, 1 );
  sendJoy();
  setButton( 0, 0 );
  sendJoy();
  advanceMs( 300 );
  sendJoy();

  // Second press, release, wait past window — also flushes as single press.
  setButton( 0, 1 );
  sendJoy();
  setButton( 0, 0 );
  sendJoy();
  advanceMs( 300 );
  sendJoy();
}

// Holding across the window: timeout flushes the press, then holds, then release on button-up.
TEST_F( HectorGamepadManagerDoublePressTest, HeldButtonAfterTimeoutGeneratesHoldsAndRelease )
{
  EXPECT_CALL( *pub_probe_press_, publish( _ ) ).Times( 0 );
  setButton( 0, 1 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  // Window expires while still held — press flushes, no release yet.
  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_release_, publish( _ ) ).Times( 0 );
  EXPECT_CALL( *pub_probe_hold_, publish( _ ) ).Times( 0 );
  advanceMs( 300 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_release_.get() );
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_hold_.get() );

  // Button 0 has no on_hold so it falls back to on_press ("probe").
  EXPECT_CALL( *pub_probe_hold_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "hold:probe:" ) ) ) )
      .Times( 2 );
  sendJoy();
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_hold_.get() );

  // Release fires when the button finally goes up.
  EXPECT_CALL( *pub_probe_release_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "release:probe:" ) ) ) )
      .Times( 1 );
  setButton( 0, 0 );
  sendJoy();
}

// Verifies hold/release dispatch uses per-event YAML function names, not the on_press fallback.
TEST_F( HectorGamepadManagerDoublePressTest, CustomHoldAndReleaseFunctionNamesAreDispatched )
{
  setButton( 1, 1 );
  sendJoy();
  advanceMs( 300 );

  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 1 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  EXPECT_CALL( *pub_probe_hold_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "hold:probe_hold:" ) ) ) )
      .Times( 1 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_hold_.get() );

  EXPECT_CALL( *pub_probe_release_, publish( Field( &std_msgs::msg::String::data,
                                                    HasSubstr( "release:probe_release:" ) ) ) )
      .Times( 1 );
  setButton( 1, 0 );
  sendJoy();
}

// Per-button trackers are independent: a press on one button must not flush the other.
TEST_F( HectorGamepadManagerDoublePressTest, InterleavedDoublePressButtonsTrackIndependently )
{
  // Press button 0, then button 2 inside button 0's window. Neither fires yet.
  EXPECT_CALL( *pub_probe_press_, publish( _ ) ).Times( 0 );
  EXPECT_CALL( *pub_probe_release_, publish( _ ) ).Times( 0 );

  setButton( 0, 1 );
  sendJoy();
  setButton( 0, 0 );
  sendJoy();

  advanceMs( 100 ); // still inside button 0's window

  setButton( 2, 1 );
  sendJoy();
  setButton( 2, 0 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_release_.get() );

  // Past button 0's window but inside button 2's: button 0 flushes, button 2 does not.
  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_release_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "release:probe:" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "probe2" ) ) ) )
      .Times( 0 );
  advanceMs( 200 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_release_.get() );

  // Second press on button 2 dispatches probe2_double; button 0 must not re-dispatch.
  EXPECT_CALL( *pub_probe_press_, publish( Field( &std_msgs::msg::String::data,
                                                  HasSubstr( "press:probe2_double:" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 0 );
  setButton( 2, 1 );
  sendJoy();
}

// Config switch while a double-press button is pressed must synthesize a release.
TEST_F( HectorGamepadManagerDoublePressTest, ConfigSwitchWhileHeldFlushesRelease )
{
  // Drive button 0 into press_dispatched=true via timeout flush while still held.
  setButton( 0, 1 );
  sendJoy();
  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 1 );
  advanceMs( 300 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  // Trigger config switch via button 7; the outgoing config's button 0 must receive a synthesized release.
  EXPECT_CALL( *pub_probe_release_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "release:probe:" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_press_, publish( _ ) ).Times( 0 );
  EXPECT_CALL( *pub_probe_hold_, publish( _ ) ).Times( 0 );

  // Release button 0 first so handleConfigurationSwitches doesn't race it.
  setButton( 0, 0 );
  setButton( 7, 1 );
  sendJoy();
}

// A backward clock jump must not strand a buffered press; negative deltas count as expired.
TEST_F( HectorGamepadManagerDoublePressTest, BackwardClockJumpDoesNotStickBufferedPress )
{
  advanceMs( 10000 );

  setButton( 0, 1 );
  sendJoy();
  setButton( 0, 0 );
  sendJoy();
  clock_->resetClock( 5000000000L ); // 5.0s, below the 10s anchor

  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_release_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "release:probe:" ) ) ) )
      .Times( 1 );
  sendJoy();
}

// A buffered tap must flush before the next rising edge overwrites last_press_time.
TEST_F( HectorGamepadManagerDoublePressTest, ExpiredBufferedPressFlushesOnNextRisingEdge )
{
  setButton( 0, 1 );
  sendJoy();
  setButton( 0, 0 );
  sendJoy();

  // Wait long past the window without any joy callback.
  advanceMs( 1000 );

  // Fresh press: the original tap flushes as press+release before the new press buffers.
  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_release_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "release:probe:" ) ) ) )
      .Times( 1 );
  setButton( 0, 1 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_release_.get() );

  // Confirm the new press is buffered: until its window expires, no press fires.
  EXPECT_CALL( *pub_probe_press_, publish( _ ) ).Times( 0 );
  setButton( 0, 0 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  // Once the new window expires, the second tap also flushes as a single press+release.
  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_release_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "release:probe:" ) ) ) )
      .Times( 1 );
  advanceMs( 300 );
  sendJoy();
}

// double_press_window_sec = 0 disables buffering entirely; on_double_press becomes unreachable.
class HectorGamepadManagerZeroWindowTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions opts_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<hector_gamepad_manager::HectorGamepadManager> manager_;
  std::unique_ptr<rtest::TestClock> clock_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> sub_joy_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_config_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_press_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_release_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_hold_;

  sensor_msgs::msg::Joy joy_msg_;

  static std::string paramsFilePath()
  {
    auto path = std::filesystem::path( __FILE__ ).parent_path() / "config" /
                "manager_internal_zero_window_params.yaml";
    return path.string();
  }

  void SetUp() override
  {
    opts_ = rclcpp::NodeOptions();
    opts_.arguments( { "--ros-args", "--params-file", paramsFilePath() } );

    node_ = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_zero_window_test", opts_ );
    clock_ = std::make_unique<rtest::TestClock>( node_ );
    manager_ = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node_ );

    sub_joy_ = rtest::findSubscription<sensor_msgs::msg::Joy>( node_, "/ocs/joy" );
    pub_config_ = rtest::findPublisher<std_msgs::msg::String>( node_, "/ocs/joy_teleop_profile" );
    pub_probe_press_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/press" );
    pub_probe_release_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/release" );
    pub_probe_hold_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/hold" );

    ASSERT_TRUE( sub_joy_ );
    ASSERT_TRUE( pub_config_ );
    ASSERT_TRUE( pub_probe_press_ );
    ASSERT_TRUE( pub_probe_release_ );
    ASSERT_TRUE( pub_probe_hold_ );

    EXPECT_CALL( *pub_config_, publish( _ ) ).Times( AnyNumber() );

    joy_msg_.axes = std::vector<float>( MAX_AXES, 0.0f );
    joy_msg_.buttons = std::vector<int>( MAX_BUTTONS, 0 );
    joy_msg_.axes[2] = 1.0f;
    joy_msg_.axes[5] = 1.0f;
  }

  void setButton( int id, int value ) { joy_msg_.buttons[id] = value; }
  void sendJoy() { sub_joy_->handle_message( joy_msg_ ); }
};

// A zero-window press flushes immediately and pairs cleanly with a release on button-up.
TEST_F( HectorGamepadManagerZeroWindowTest, ZeroWindowFlushesPressImmediately )
{
  // Press flushes immediately; button still held leaves press_dispatched=true.
  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_release_, publish( _ ) ).Times( 0 );
  setButton( 0, 1 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_release_.get() );

  // Release fires when the button goes up.
  EXPECT_CALL( *pub_probe_release_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "release:probe:" ) ) ) )
      .Times( 1 );
  setButton( 0, 0 );
  sendJoy();
}
