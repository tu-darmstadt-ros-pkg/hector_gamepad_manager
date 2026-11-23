#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>
#include <rtest/create_timer_mock.hpp>
#include <rtest/publisher_mock.hpp>
#include <rtest/subscription_mock.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <std_msgs/msg/string.hpp>

#include <hector_gamepad_manager/hector_gamepad_manager.hpp>

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

constexpr int MAX_BUTTONS =
    21; // SDL GameController layout; axis-derived buttons are synthesized internally
constexpr int MAX_AXES = 6;

class HectorGamepadManagerInternalTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions opts_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<hector_gamepad_manager::HectorGamepadManager> manager_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> sub_joy_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_config_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_press_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_hold_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_release_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_axis_;
  std::shared_ptr<rtest::PublisherMock<sensor_msgs::msg::JoyFeedback>> pub_feedback_;

  sensor_msgs::msg::Joy joy_msg_;
  std::map<std::string, int> button_map_;
  std::map<std::string, int> axis_map_;

  static std::string paramsFilePath()
  {
    auto path =
        std::filesystem::path( __FILE__ ).parent_path() / "config" / "manager_internal_params.yaml";
    return path.string();
  }

  void SetUp() override
  {
    opts_ = rclcpp::NodeOptions();
    opts_.arguments( { "--ros-args", "--params-file", paramsFilePath() } );

    node_ = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_internal_test", "athena", opts_ );
    manager_ = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node_ );

    sub_joy_ = rtest::findSubscription<sensor_msgs::msg::Joy>( node_, "/athena/joy" );
    pub_config_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/joy_teleop_profile" );
    pub_probe_press_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/press" );
    pub_probe_hold_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/hold" );
    pub_probe_release_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/release" );
    pub_probe_axis_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/axis" );
    pub_feedback_ =
        rtest::findPublisher<sensor_msgs::msg::JoyFeedback>( node_, "/athena/joy_feedback" );

    ASSERT_TRUE( sub_joy_ );
    ASSERT_TRUE( pub_feedback_ );
    ASSERT_TRUE( pub_config_ );
    ASSERT_TRUE( pub_probe_press_ );
    ASSERT_TRUE( pub_probe_hold_ );
    ASSERT_TRUE( pub_probe_release_ );
    ASSERT_TRUE( pub_probe_axis_ );

    EXPECT_CALL( *pub_config_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
    EXPECT_CALL( *pub_probe_press_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
    EXPECT_CALL( *pub_probe_hold_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
    EXPECT_CALL( *pub_probe_release_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
    EXPECT_CALL( *pub_probe_axis_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
    EXPECT_CALL( *pub_feedback_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

    // SDL GameController layout, i.e. what game_controller_node publishes.
    button_map_ = { { "a", 0 }, { "back", 4 }, { "start", 6 }, { "share", 15 } };
    axis_map_ = { { "left_stick_left_right", 0 }, { "left_trigger", 4 } };

    resetJoy();
  }

  void resetJoy()
  {
    joy_msg_.axes = std::vector<float>( MAX_AXES, 0.0f );
    joy_msg_.buttons = std::vector<int>( MAX_BUTTONS, 0 );
  }

  void setButton( const std::string &button, int value, bool do_reset = false )
  {
    if ( do_reset ) {
      resetJoy();
    }
    joy_msg_.buttons[button_map_[button]] = value;
  }

  void setAxis( const std::string &axis, float value, bool do_reset = false )
  {
    if ( do_reset ) {
      resetJoy();
    }
    joy_msg_.axes[axis_map_[axis]] = value;
  }

  void sendJoy() { sub_joy_->handle_message( joy_msg_ ); }

  // Fire the FeedbackManager's periodic publish timer once.
  void fireFeedbackTimer()
  {
    for ( auto &timer : rtest::findTimers( node_ ) ) {
      timer->execute_callback( std::make_shared<int>( 0 ) );
    }
  }
};

namespace
{
std::shared_ptr<rclcpp::Node> makeNodeWithParams( const std::string &node_name,
                                                  const std::string &params_filename )
{
  const auto params = std::filesystem::path( __FILE__ ).parent_path() / "config" / params_filename;
  rclcpp::NodeOptions opts;
  opts.arguments( { "--ros-args", "--params-file", params.string() } );
  return std::make_shared<rclcpp::Node>( node_name, "athena", opts );
}
} // namespace

// The two button sections are not interchangeable: "buttons" holds controls the gamepad reports,
// "axis_buttons" holds ones synthesized from a deflected axis. A name in the wrong section is a
// config error and must be rejected rather than silently ignored.
TEST( HectorGamepadManagerConfigValidation, RejectsMisplacedButtonNames )
{
  auto node = makeNodeWithParams( "gamepad_manager_misplaced_name_test",
                                  "manager_internal_misplaced_name_params.yaml" );
  auto manager = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node );
  EXPECT_FALSE( rtest::findSubscription<sensor_msgs::msg::Joy>( node, "/athena/joy" ) );
}

TEST( HectorGamepadManagerConfigValidation, RejectsUnknownAxisButtonNames )
{
  auto node = makeNodeWithParams( "gamepad_manager_bad_axis_name_test",
                                  "manager_internal_bad_axis_name_params.yaml" );
  auto manager = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node );
  EXPECT_FALSE( rtest::findSubscription<sensor_msgs::msg::Joy>( node, "/athena/joy" ) );
}

TEST_F( HectorGamepadManagerInternalTest, ButtonHoldAndReleaseSequence )
{
  EXPECT_CALL( *pub_probe_press_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "press:probe" ) ) ) )
      .Times( 1 );
  setButton( "a", 1, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  EXPECT_CALL( *pub_probe_hold_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "hold:probe" ) ) ) )
      .Times( 1 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_hold_.get() );

  EXPECT_CALL( *pub_probe_release_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "release:probe" ) ) ) )
      .Times( 1 );
  setButton( "a", 0, true );
  sendJoy();
}

// A plain button held when the config goes away must still get its release: the manager detects
// the edges, so nothing else would ever end the press it already dispatched.
TEST_F( HectorGamepadManagerInternalTest, ConfigSwitchWhileHeldReleasesAPlainButton )
{
  EXPECT_CALL( *pub_probe_press_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "press:probe" ) ) ) )
      .Times( 1 );
  setButton( "a", 1, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  // The switch button goes down while "a" is still held, so the release can only come from the
  // flush: handleConfigurationSwitches returns before the button loop runs.
  EXPECT_CALL( *pub_probe_release_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "release:probe" ) ) ) )
      .Times( 1 );
  EXPECT_CALL( *pub_probe_hold_, publish( ::testing::_ ) ).Times( 0 );
  setButton( "back", 1 ); // switch to manager_internal_alt
  sendJoy();
}

// SDL reports the d-pad as four real buttons. `dpad_up` is mapped in manager_internal.yaml.
TEST_F( HectorGamepadManagerInternalTest, DpadButtonDispatches )
{
  EXPECT_CALL( *pub_probe_press_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "press:dpad" ) ) ) )
      .Times( 1 );
  resetJoy();
  joy_msg_.buttons[11] = 1; // dpad_up
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  EXPECT_CALL( *pub_probe_release_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "release:dpad" ) ) ) )
      .Times( 1 );
  joy_msg_.buttons[11] = 0;
  sendJoy();
}

TEST_F( HectorGamepadManagerInternalTest, ShareButtonDispatches )
{
  EXPECT_CALL( *pub_probe_press_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "press:share" ) ) ) )
      .Times( 1 );
  setButton( "share", 1, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  EXPECT_CALL( *pub_probe_release_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "release:share" ) ) ) )
      .Times( 1 );
  setButton( "share", 0, true );
  sendJoy();
}

// Controls only some pads report - the rear paddles - map 1:1 like any other button;
// `paddle1` (id 16) is mapped in manager_internal.yaml.
TEST_F( HectorGamepadManagerInternalTest, ExtraPhysicalButtonDispatches )
{
  EXPECT_CALL( *pub_probe_press_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "press:extra" ) ) ) )
      .Times( 1 );
  resetJoy();
  joy_msg_.buttons[16] = 1;
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  EXPECT_CALL( *pub_probe_release_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "release:extra" ) ) ) )
      .Times( 1 );
  joy_msg_.buttons[16] = 0;
  sendJoy();
}

// The joy source is checked on every message, not just the first: it can be relaunched, or joined
// by a second publisher, while the manager runs.
//
// Reporting is throttled through a logging macro whose state is a process-wide static, so only the
// first test in this binary to trip the check can observe the message. That is why all of it lives
// in one test, and why this test must stay ahead of the others that send a short message.
TEST_F( HectorGamepadManagerInternalTest, ReportsAWrongJoySourceAndHowToFixIt )
{
  // game_controller_node's layout passes silently, whatever the pad.
  testing::internal::CaptureStderr();
  resetJoy();
  sendJoy();
  EXPECT_THAT( testing::internal::GetCapturedStderr(),
               ::testing::Not( ::testing::HasSubstr( "game_controller_node" ) ) );

  // joy_node sizes per device - an Xbox pad gives 8 axes and 11 buttons - so the counts alone
  // identify the wrong source before any control is touched. A good first message must not have
  // switched the check off.
  testing::internal::CaptureStderr();
  joy_msg_.axes.resize( 8 );
  joy_msg_.buttons.resize( 11 );
  sendJoy();
  const std::string log = testing::internal::GetCapturedStderr();
  EXPECT_THAT( log, ::testing::HasSubstr( "game_controller_node" ) );
  EXPECT_THAT( log, ::testing::HasSubstr( "joy_node" ) );

  // A stream of bad messages reports once, not once per message.
  testing::internal::CaptureStderr();
  sendJoy();
  EXPECT_THAT( testing::internal::GetCapturedStderr(),
               ::testing::Not( ::testing::HasSubstr( "always publishes" ) ) );
}

// A message of another layout is dropped, not dispatched. Its ids address different controls than
// the config was written against, so acting on the ones that happen to be in range would command
// whatever shares the index - the reason this is not "read what is there and pad the rest".
TEST_F( HectorGamepadManagerInternalTest, JoyMessageOfAnotherLayoutIsDropped )
{
  // Not even a button that is present in the short message: id 0 is "a" to this manager, but in
  // the layout that produced the message it is some other control. Same for the config switch,
  // which is the one an operator would notice - a wrong source must not change the mode either.
  EXPECT_CALL( *pub_probe_press_, publish( ::testing::_ ) ).Times( 0 );
  EXPECT_CALL( *pub_config_, publish( ::testing::_ ) ).Times( 0 );
  setButton( "a", 1, true );
  setButton( "back", 1 );        // switches config, in a message the manager accepts
  joy_msg_.buttons.resize( 11 ); // no Share button
  joy_msg_.axes.resize( 2 );     // sticks only, no triggers / cross
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_config_.get() );

  // The same press in a message of the expected layout does switch, so what the check rejects is
  // the layout and not the press.
  EXPECT_CALL( *pub_config_,
               publish( ::testing::Field( &std_msgs::msg::String::data, "manager_internal_alt" ) ) )
      .Times( 1 );
  setButton( "back", 1, true );
  sendJoy();
}

// A pad without paddles or a touchpad reports fewer buttons and is still game_controller_node, so
// its messages are dispatched. Reading one must stay in bounds: the ids it does not carry count as
// "never pressed" rather than indexing past the end of the message.
TEST_F( HectorGamepadManagerInternalTest, PadWithoutExtraButtonsDispatchesAndReadsMissingAsNeutral )
{
  // paddle1 (id 16) is bound in this config and absent from the message; "a" is present.
  EXPECT_CALL( *pub_probe_press_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "press:extra" ) ) ) )
      .Times( 0 );
  EXPECT_CALL( *pub_probe_press_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "press:probe" ) ) ) )
      .Times( 1 );
  setButton( "a", 1, true );
  joy_msg_.buttons.resize( 15 ); // the standard buttons, no paddles and no touchpad
  sendJoy();
}

// With config_switch_vibration_enabled: false, a config switch must not publish any rumble.
TEST( HectorGamepadManagerRumble, DisabledViaParameter )
{
  auto node = makeNodeWithParams( "gamepad_manager_no_rumble_test",
                                  "manager_internal_no_rumble_params.yaml" );
  auto manager = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node );
  auto sub_joy = rtest::findSubscription<sensor_msgs::msg::Joy>( node, "/athena/joy" );
  auto pub_feedback =
      rtest::findPublisher<sensor_msgs::msg::JoyFeedback>( node, "/athena/joy_feedback" );
  ASSERT_TRUE( sub_joy );
  ASSERT_TRUE( pub_feedback );

  EXPECT_CALL( *pub_feedback, publish( ::testing::_ ) ).Times( 0 );
  sensor_msgs::msg::Joy joy_msg;
  joy_msg.axes = std::vector<float>( MAX_AXES, 0.0f );
  joy_msg.buttons = std::vector<int>( MAX_BUTTONS, 0 );
  joy_msg.buttons[4] = 1; // Back: switch to manager_internal_alt
  sub_joy->handle_message( joy_msg );
  for ( auto &timer : rtest::findTimers( node ) ) {
    timer->execute_callback( std::make_shared<int>( 0 ) );
  }
}

// A config switch fires a short one-shot rumble so the operator feels the mode change.
// The initial config activation at startup must not rumble.
TEST_F( HectorGamepadManagerInternalTest, ConfigSwitchTriggersRumble )
{
  // Startup (default config already active): feedback stays idle, nothing is published.
  EXPECT_CALL( *pub_feedback_, publish( ::testing::_ ) ).Times( 0 );
  fireFeedbackTimer();
  ::testing::Mock::VerifyAndClearExpectations( pub_feedback_.get() );

  // Switching to another config publishes a rumble with non-zero intensity.
  EXPECT_CALL( *pub_feedback_, publish( ::testing::Field( &sensor_msgs::msg::JoyFeedback::intensity,
                                                          ::testing::Gt( 0.0f ) ) ) )
      .Times( 1 );
  setButton( "back", 1, true ); // switch to manager_internal_alt
  sendJoy();
  fireFeedbackTimer();
}

TEST_F( HectorGamepadManagerInternalTest, ConfigSwitchBlocksOtherInputs )
{
  EXPECT_CALL( *pub_probe_axis_, publish( ::testing::_ ) ).Times( 0 );
  EXPECT_CALL( *pub_config_,
               publish( ::testing::Field( &std_msgs::msg::String::data, "manager_internal_alt" ) ) )
      .Times( 1 );
  setAxis( "left_stick_left_right", 1.0f, true );
  setButton( "back", 1 );
  sendJoy();
}

// The virtual button synthesized from a trigger reads the normalized value, so it fires once the
// trigger is pressed past the deadzone.
TEST_F( HectorGamepadManagerInternalTest, TriggerVirtualButtonFiresWhenPressed )
{
  EXPECT_CALL( *pub_probe_axis_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

  // Inside the deadzone: no press.
  EXPECT_CALL( *pub_probe_press_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "press:trigger_button" ) ) ) )
      .Times( 0 );
  setAxis( "left_trigger", -0.4f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  EXPECT_CALL( *pub_probe_axis_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  EXPECT_CALL( *pub_probe_press_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "press:trigger_button" ) ) ) )
      .Times( 1 );
  setAxis( "left_trigger", -0.6f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  EXPECT_CALL( *pub_probe_axis_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
  EXPECT_CALL( *pub_probe_release_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "release:trigger_button" ) ) ) )
      .Times( 1 );
  setAxis( "left_trigger", 0.0f, true );
  sendJoy();
}

TEST_F( HectorGamepadManagerInternalTest, AxisDeadzoneMapsToVirtualButton )
{
  EXPECT_CALL( *pub_probe_press_, publish( ::testing::_ ) ).Times( 0 );
  setAxis( "left_stick_left_right", 0.4f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  EXPECT_CALL( *pub_probe_press_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "press:virtual" ) ) ) )
      .Times( 1 );
  setAxis( "left_stick_left_right", 0.6f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  EXPECT_CALL( *pub_probe_hold_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "hold:virtual" ) ) ) )
      .Times( 1 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_hold_.get() );

  EXPECT_CALL( *pub_probe_release_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "release:virtual" ) ) ) )
      .Times( 1 );
  setAxis( "left_stick_left_right", 0.0f, true );
  sendJoy();
}
