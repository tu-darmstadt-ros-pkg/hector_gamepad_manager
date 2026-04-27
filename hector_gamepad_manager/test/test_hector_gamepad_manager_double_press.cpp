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

// Test A — Quick tap with on_double_press configured: a press immediately followed by a release
// must, after the double-press window expires, dispatch on_press AND a paired on_release.
// Without the fix, on_release is never sent, leaving plugins stuck in the "pressed" state.
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

// Test C — A press flushed by timeout, then a second fresh press: each should fire as a single
// press, never as a double-press.
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

// Test D — Holding the button across the window boundary: timeout flushes the press, subsequent
// frames dispatch on_hold, and finally a release fires when the button goes up.
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

  // Subsequent frames with the button still held → hold (button 0 has no on_hold, so it falls
  // back to on_press's function name "probe").
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

// Test D2 — Same as D, but on a button with explicit on_hold/on_release function names. Verifies
// the dispatched function name comes from the per-event YAML keys, not from on_press fallback.
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
