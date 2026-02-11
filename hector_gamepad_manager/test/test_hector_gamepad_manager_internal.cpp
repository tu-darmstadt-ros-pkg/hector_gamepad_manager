#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>
#include <rtest/publisher_mock.hpp>
#include <rtest/subscription_mock.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include <hector_gamepad_manager/hector_gamepad_manager.hpp>

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

constexpr int MAX_BUTTONS = 25;
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

    node_ = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_internal_test", opts_ );
    manager_ = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node_ );

    sub_joy_ = rtest::findSubscription<sensor_msgs::msg::Joy>( node_, "/ocs/joy" );
    pub_config_ = rtest::findPublisher<std_msgs::msg::String>( node_, "/ocs/joy_teleop_profile" );
    pub_probe_press_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/press" );
    pub_probe_hold_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/hold" );
    pub_probe_release_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/release" );
    pub_probe_axis_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/axis" );

    ASSERT_TRUE( sub_joy_ );
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

    button_map_ = { { "a", 0 }, { "back", 6 }, { "start", 7 } };
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
};

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
