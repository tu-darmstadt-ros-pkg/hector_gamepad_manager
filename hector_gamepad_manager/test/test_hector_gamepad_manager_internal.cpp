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
    12; // physical buttons on the wire; axis-derived buttons are synthesized internally
constexpr int MAX_AXES = 8;

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

    button_map_ = { { "a", 0 }, { "back", 6 }, { "start", 7 }, { "share", 11 } };
    axis_map_ = { { "left_stick_left_right", 0 } };

    resetJoy();
  }

  void resetJoy()
  {
    joy_msg_.axes = std::vector<float>( MAX_AXES, 0.0f );
    joy_msg_.buttons = std::vector<int>( MAX_BUTTONS, 0 );
    joy_msg_.axes[2] = 1.0f;
    joy_msg_.axes[5] = 1.0f;
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

// Physical button ids inside the virtual button range (>= 32) would collide with the axis-derived
// buttons. Such entries must be skipped with a warning while the rest of the config still loads.
TEST( HectorGamepadManagerConfigValidation, SkipsButtonIdsOverlappingVirtualRange )
{
  auto node = makeNodeWithParams( "gamepad_manager_overlapping_ids_test",
                                  "manager_internal_overlapping_ids_params.yaml" );
  auto manager = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node );
  EXPECT_TRUE( rtest::findSubscription<sensor_msgs::msg::Joy>( node, "/athena/joy" ) );
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

TEST_F( HectorGamepadManagerInternalTest, ShareButtonMapsToButton11 )
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

// Gamepads with more physical buttons than the Xbox layout (e.g. rear paddles) map 1:1 without
// any code changes; button 13 is mapped in manager_internal.yaml.
TEST_F( HectorGamepadManagerInternalTest, ExtraPhysicalButtonDispatches )
{
  EXPECT_CALL( *pub_probe_press_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "press:extra" ) ) ) )
      .Times( 1 );
  resetJoy();
  joy_msg_.buttons.resize( 14, 0 );
  joy_msg_.buttons[13] = 1;
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  EXPECT_CALL( *pub_probe_release_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "release:extra" ) ) ) )
      .Times( 1 );
  joy_msg_.buttons[13] = 0;
  sendJoy();
}

// Gamepads without a Share button publish only 11 buttons (older pads even fewer axes).
// Entries missing from the message must read as "never pressed" instead of crashing.
TEST_F( HectorGamepadManagerInternalTest, ShortJoyMessageTreatsMissingEntriesAsNeutral )
{
  EXPECT_CALL( *pub_probe_press_, publish( ::testing::_ ) ).Times( 0 );
  resetJoy();
  joy_msg_.buttons.resize( 11 ); // no Share button
  joy_msg_.axes.resize( 2 );     // sticks only, no triggers / cross
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  // Buttons that do exist in the short message still dispatch normally.
  EXPECT_CALL( *pub_probe_press_,
               publish( ::testing::Field( &std_msgs::msg::String::data,
                                          ::testing::HasSubstr( "press:probe" ) ) ) )
      .Times( 1 );
  setButton( "a", 1, true );
  joy_msg_.buttons.resize( 11 );
  joy_msg_.axes.resize( 2 );
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
  joy_msg.axes[2] = 1.0f;
  joy_msg.axes[5] = 1.0f;
  joy_msg.buttons = std::vector<int>( MAX_BUTTONS, 0 );
  joy_msg.buttons[6] = 1; // Back: switch to manager_internal_alt
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
