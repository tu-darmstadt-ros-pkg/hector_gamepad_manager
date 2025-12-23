#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>
#include <rtest/publisher_mock.hpp>
#include <rtest/subscription_mock.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <hector_gamepad_manager/hector_gamepad_manager.hpp>

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

constexpr int MAX_BUTTONS = 25;
constexpr int MAX_AXES = 8;

class HectorGamepadManagerTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions opts_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<hector_gamepad_manager::HectorGamepadManager> manager_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> sub_joy_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_config_;
  std::shared_ptr<rtest::PublisherMock<geometry_msgs::msg::TwistStamped>> pub_cmd_vel_;
  std::shared_ptr<rtest::PublisherMock<geometry_msgs::msg::TwistStamped>> pub_twist_eef_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::Float64>> pub_gripper_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::Float64MultiArray>> pub_flipper_;

  sensor_msgs::msg::Joy joy_msg_;
  std::map<std::string, int> button_map_;
  std::map<std::string, int> axis_map_;
  std::string active_config_;

  static std::string paramsFilePath()
  {
    auto path =
        std::filesystem::path( __FILE__ ).parent_path() / "config" / "athena_plugin_config.yaml";
    return path.string();
  }

  void SetUp() override
  {
    opts_ = rclcpp::NodeOptions();
    opts_.arguments( { "--ros-args", "--params-file", paramsFilePath() } );

    node_ = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_test", opts_ );
    manager_ = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node_ );

    sub_joy_ = rtest::findSubscription<sensor_msgs::msg::Joy>( node_, "/ocs/joy" );
    pub_config_ = rtest::findPublisher<std_msgs::msg::String>( node_, "/ocs/active_config" );
    pub_cmd_vel_ =
        rtest::findPublisher<geometry_msgs::msg::TwistStamped>( node_, "/athena/cmd_vel" );
    pub_twist_eef_ = rtest::findPublisher<geometry_msgs::msg::TwistStamped>(
        node_, "/athena/moveit_twist_controller/eef_cmd" );
    pub_gripper_ = rtest::findPublisher<std_msgs::msg::Float64>(
        node_, "/athena/moveit_twist_controller/gripper_vel_cmd" );
    pub_flipper_ = rtest::findPublisher<std_msgs::msg::Float64MultiArray>(
        node_, "/athena/flipper_velocity_controller/commands" );

    ASSERT_TRUE( sub_joy_ );
    ASSERT_TRUE( pub_config_ );
    ASSERT_TRUE( pub_cmd_vel_ );
    ASSERT_TRUE( pub_twist_eef_ );
    ASSERT_TRUE( pub_gripper_ );
    ASSERT_TRUE( pub_flipper_ );

    EXPECT_CALL( *pub_config_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
    EXPECT_CALL( *pub_cmd_vel_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
    EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
    EXPECT_CALL( *pub_gripper_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );
    EXPECT_CALL( *pub_flipper_, publish( ::testing::_ ) ).Times( ::testing::AnyNumber() );

    button_map_ = { { "a", 0 },
                    { "b", 1 },
                    { "x", 2 },
                    { "y", 3 },
                    { "lb", 4 },
                    { "rb", 5 },
                    { "back", 6 },
                    { "start", 7 },
                    { "power", 8 },
                    { "left_joy", 9 },
                    { "right_joy", 10 },
                    { "cross_left", 21 },
                    { "cross_right", 22 },
                    { "cross_up", 23 },
                    { "cross_down", 24 } };
    axis_map_ = { { "left_stick_left_right", 0 },  { "left_stick_up_down", 1 },  { "lt", 2 },
                  { "right_stick_left_right", 3 }, { "right_stick_up_down", 4 }, { "rt", 5 },
                  { "cross_left_right", 6 },       { "cross_up_down", 7 } };

    resetJoy();
    active_config_ = "driving";
  }

  void resetJoy()
  {
    joy_msg_.axes = std::vector<float>( MAX_AXES, 0.0f );
    joy_msg_.buttons = std::vector<int>( MAX_BUTTONS, 0 );
    joy_msg_.axes[axis_map_["rt"]] = 1.0f;
    joy_msg_.axes[axis_map_["lt"]] = 1.0f;
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
    if ( axis == "lt" || axis == "rt" ) {
      value = -value;
    }
    joy_msg_.axes[axis_map_[axis]] = value;
  }

  void sendJoy() { sub_joy_->handle_message( joy_msg_ ); }

  void expectActiveConfig( const std::string &config )
  {
    EXPECT_CALL( *pub_config_, publish( ::testing::Field( &std_msgs::msg::String::data, config ) ) )
        .Times( 1 )
        .WillOnce( [this]( const std_msgs::msg::String &msg ) { active_config_ = msg.data; } );
  }

  void switchToConfig( const std::string &config )
  {
    if ( config == active_config_ ) {
      return;
    }
    if ( config == "manipulation" ) {
      setButton( "back", 1, true );
      expectActiveConfig( "manipulation" );
      sendJoy();
      ::testing::Mock::VerifyAndClearExpectations( pub_config_.get() );
    } else if ( config == "driving" ) {
      setButton( "start", 1, true );
      expectActiveConfig( "driving" );
      sendJoy();
      ::testing::Mock::VerifyAndClearExpectations( pub_config_.get() );
    } else {
      FAIL() << "Unexpected config name: " << config;
    }
    resetJoy();
  }
};

TEST_F( HectorGamepadManagerTest, SwitchConfig )
{
  ::testing::Sequence seq;
  EXPECT_CALL( *pub_config_,
               publish( ::testing::Field( &std_msgs::msg::String::data, "manipulation" ) ) )
      .Times( 1 )
      .InSequence( seq )
      .WillOnce( [this]( const std_msgs::msg::String &msg ) { active_config_ = msg.data; } );
  EXPECT_CALL( *pub_config_,
               publish( ::testing::Field( &std_msgs::msg::String::data, "driving" ) ) )
      .Times( 1 )
      .InSequence( seq )
      .WillOnce( [this]( const std_msgs::msg::String &msg ) { active_config_ = msg.data; } );

  setButton( "back", 1, true );
  sendJoy();
  setButton( "start", 1, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_config_.get() );

  EXPECT_CALL( *pub_config_, publish( ::testing::_ ) ).Times( 0 );
  setButton( "start", 1, true );
  sendJoy();
}

TEST_F( HectorGamepadManagerTest, CmdVel )
{
  switchToConfig( "driving" );
  EXPECT_CALL( *pub_cmd_vel_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_GE( msg.twist.linear.x, 0.0 );
        EXPECT_EQ( msg.twist.linear.y, 0.0 );
        EXPECT_EQ( msg.twist.linear.z, 0.0 );
        EXPECT_EQ( msg.twist.angular.x, 0.0 );
        EXPECT_EQ( msg.twist.angular.y, 0.0 );
        EXPECT_NE( msg.twist.angular.z, 0.0 );
      } );

  setAxis( "left_stick_left_right", 0.5f, true );
  setAxis( "left_stick_up_down", 0.5f );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_cmd_vel_.get() );

  EXPECT_CALL( *pub_cmd_vel_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_LT( msg.twist.linear.x, 0.0 );
        EXPECT_EQ( msg.twist.linear.y, 0.0 );
        EXPECT_EQ( msg.twist.linear.z, 0.0 );
        EXPECT_EQ( msg.twist.angular.x, 0.0 );
        EXPECT_EQ( msg.twist.angular.y, 0.0 );
        EXPECT_EQ( msg.twist.angular.z, 0.0 );
      } );

  setAxis( "left_stick_up_down", -0.5f, true );
  sendJoy();
}

TEST_F( HectorGamepadManagerTest, SpeedButtons )
{
  switchToConfig( "driving" );
  constexpr double max_linear_speed = 1.1;
  constexpr double max_angular_speed = 1.5;
  constexpr double normal_factor = 1.0;
  constexpr double fast_factor = 2.0;
  constexpr double slow_factor = 0.3;

  EXPECT_CALL( *pub_cmd_vel_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.linear.x, normal_factor * max_linear_speed, 0.1 );
        EXPECT_NEAR( msg.twist.angular.z, normal_factor * max_angular_speed, 0.1 );
      } );
  setAxis( "left_stick_left_right", 1.0f, true );
  setAxis( "left_stick_up_down", 1.0f );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_cmd_vel_.get() );

  EXPECT_CALL( *pub_cmd_vel_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.linear.x, slow_factor * max_linear_speed, 0.1 );
        EXPECT_NEAR( msg.twist.angular.z, slow_factor * max_angular_speed, 0.1 );
      } );
  setAxis( "left_stick_left_right", 1.0f, true );
  setAxis( "left_stick_up_down", 1.0f );
  setButton( "x", 1 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_cmd_vel_.get() );

  EXPECT_CALL( *pub_cmd_vel_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.linear.x, fast_factor * max_linear_speed, 0.1 );
        EXPECT_NEAR( msg.twist.angular.z, fast_factor * max_angular_speed, 0.1 );
      } );
  setAxis( "left_stick_left_right", 1.0f, true );
  setAxis( "left_stick_up_down", 1.0f );
  setButton( "a", 1 );
  setButton( "x", 0 );
  sendJoy();
}

TEST_F( HectorGamepadManagerTest, NoMoreCmdVelTwistWhenSwitchingToManipulationConfig )
{
  switchToConfig( "driving" );
  EXPECT_CALL( *pub_cmd_vel_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NE( msg.twist.linear.x, 0.0 );
      } );
  setAxis( "left_stick_left_right", 1.0f, true );
  setAxis( "left_stick_up_down", 1.0f );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_cmd_vel_.get() );

  EXPECT_CALL( *pub_cmd_vel_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_EQ( msg.twist.linear.x, 0.0 );
        EXPECT_EQ( msg.twist.angular.z, 0.0 );
      } );
  expectActiveConfig( "manipulation" );
  setButton( "back", 1, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_cmd_vel_.get() );
  ::testing::Mock::VerifyAndClearExpectations( pub_config_.get() );

  EXPECT_CALL( *pub_cmd_vel_, publish( ::testing::_ ) ).Times( 0 );
  resetJoy();
  sendJoy();
}

TEST_F( HectorGamepadManagerTest, EefTwistLinear )
{
  switchToConfig( "manipulation" );
  constexpr double max_linear_speed = 0.15;

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.linear.y, max_linear_speed, 0.01 );
      } );
  setAxis( "left_stick_left_right", 1.0f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_twist_eef_.get() );

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.linear.z, max_linear_speed, 0.01 );
      } );
  setAxis( "left_stick_up_down", 1.0f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_twist_eef_.get() );

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.linear.x, max_linear_speed, 0.01 );
      } );
  setAxis( "rt", 1.0f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_twist_eef_.get() );

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.linear.y, -max_linear_speed, 0.01 );
      } );
  setAxis( "left_stick_left_right", -1.0f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_twist_eef_.get() );

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.linear.z, -max_linear_speed, 0.01 );
      } );
  setAxis( "left_stick_up_down", -1.0f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_twist_eef_.get() );

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.linear.x, -max_linear_speed, 0.01 );
      } );
  setAxis( "lt", 1.0f, true );
  sendJoy();
}

TEST_F( HectorGamepadManagerTest, EefTwistAngular )
{
  switchToConfig( "manipulation" );
  constexpr double max_angular_speed = 0.25;

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.angular.z, max_angular_speed, 0.01 );
      } );
  setAxis( "right_stick_left_right", 1.0f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_twist_eef_.get() );

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.angular.y, max_angular_speed, 0.01 );
      } );
  setAxis( "right_stick_up_down", 1.0f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_twist_eef_.get() );

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.angular.x, max_angular_speed, 0.01 );
      } );
  setButton( "rb", 1, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_twist_eef_.get() );

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.angular.z, -max_angular_speed, 0.01 );
      } );
  setAxis( "right_stick_left_right", -1.0f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_twist_eef_.get() );

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.angular.y, -max_angular_speed, 0.01 );
      } );
  setAxis( "right_stick_up_down", -1.0f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_twist_eef_.get() );

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NEAR( msg.twist.angular.x, -max_angular_speed, 0.01 );
      } );
  setButton( "lb", 1, true );
  sendJoy();
}

TEST_F( HectorGamepadManagerTest, SendZeroEefTwistWhenActivatingDriving )
{
  switchToConfig( "manipulation" );
  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_NE( msg.twist.angular.z, 0.0 );
      } );
  setAxis( "right_stick_left_right", 1.0f, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_twist_eef_.get() );

  EXPECT_CALL( *pub_twist_eef_, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_EQ( msg.twist.linear.x, 0.0 );
        EXPECT_EQ( msg.twist.linear.y, 0.0 );
        EXPECT_EQ( msg.twist.linear.z, 0.0 );
        EXPECT_EQ( msg.twist.angular.x, 0.0 );
        EXPECT_EQ( msg.twist.angular.y, 0.0 );
        EXPECT_EQ( msg.twist.angular.z, 0.0 );
      } );
  expectActiveConfig( "driving" );
  setButton( "start", 1, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_twist_eef_.get() );
  ::testing::Mock::VerifyAndClearExpectations( pub_config_.get() );
}

TEST_F( HectorGamepadManagerTest, GripperFunctions )
{
  switchToConfig( "manipulation" );
  constexpr double gripper_speed = 0.2;

  EXPECT_CALL( *pub_gripper_, publish( ::testing::_ ) )
      .WillOnce( [gripper_speed]( const std_msgs::msg::Float64 &msg ) {
        EXPECT_EQ( msg.data, gripper_speed );
      } );
  setButton( "y", 1, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_gripper_.get() );

  EXPECT_CALL( *pub_gripper_, publish( ::testing::_ ) )
      .WillOnce( [gripper_speed]( const std_msgs::msg::Float64 &msg ) {
        EXPECT_EQ( msg.data, -gripper_speed );
      } );
  setButton( "x", 1, true );
  sendJoy();
}

TEST_F( HectorGamepadManagerTest, ZeroGripperWhenSwitchingToDriving )
{
  switchToConfig( "manipulation" );
  constexpr double gripper_speed = 0.2;

  EXPECT_CALL( *pub_gripper_, publish( ::testing::_ ) )
      .WillOnce( [gripper_speed]( const std_msgs::msg::Float64 &msg ) {
        EXPECT_EQ( msg.data, gripper_speed );
      } );
  setButton( "y", 1, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_gripper_.get() );

  EXPECT_CALL( *pub_gripper_, publish( ::testing::_ ) )
      .WillOnce( []( const std_msgs::msg::Float64 &msg ) { EXPECT_EQ( msg.data, 0.0 ); } );
  expectActiveConfig( "driving" );
  setButton( "start", 1, true );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_gripper_.get() );
  ::testing::Mock::VerifyAndClearExpectations( pub_config_.get() );
}

TEST_F( HectorGamepadManagerTest, FlipperCmdsReceived )
{
  switchToConfig( "driving" );
  constexpr double flipper_speed = 1.5;

  EXPECT_CALL( *pub_flipper_, publish( ::testing::_ ) )
      .WillOnce( [flipper_speed]( const std_msgs::msg::Float64MultiArray &msg ) {
        ASSERT_EQ( msg.data.size(), 4u );
        for ( size_t i = 0; i < 4; ++i ) { EXPECT_EQ( msg.data[i], flipper_speed ); }
      } );
  setButton( "lb", 1, true );
  setButton( "rb", 1 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_flipper_.get() );

  EXPECT_CALL( *pub_flipper_, publish( ::testing::_ ) )
      .WillOnce( [flipper_speed]( const std_msgs::msg::Float64MultiArray &msg ) {
        ASSERT_EQ( msg.data.size(), 4u );
        for ( size_t i = 0; i < 4; ++i ) { EXPECT_EQ( msg.data[i], -flipper_speed ); }
      } );
  setAxis( "rt", 1.0f, true );
  setAxis( "lt", 1.0f );
  sendJoy();
}
