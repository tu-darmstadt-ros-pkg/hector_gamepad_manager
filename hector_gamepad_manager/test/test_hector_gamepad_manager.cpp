//
// Created by aljoscha-schmidt on 3/27/25.
//
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gtest/gtest.h>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

constexpr int MAX_BUTTONS = 25;
constexpr int MAX_AXES = 8;
constexpr double TIMEOUT = 10.0;

class FakeJoyNode final : public rclcpp::Node
{
public:
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Joy joy_msg_;
  std::map<std::string, int> button_map_;
  std::map<std::string, int> axis_map_;

  FakeJoyNode() : Node( "FakeJoyNode" )
  {
    pub_ = create_publisher<sensor_msgs::msg::Joy>( "/ocs/joy", 1 );
    button_map_["a"] = 0;
    button_map_["b"] = 1;
    button_map_["x"] = 2;
    button_map_["y"] = 3;
    button_map_["lb"] = 4;
    button_map_["rb"] = 5;
    button_map_["back"] = 6;
    button_map_["start"] = 7;
    button_map_["power"] = 8;
    button_map_["left_joystick_press"] = 9;
    button_map_["right_joystick_press"] = 10;
    button_map_["cross_left"] = 21;
    button_map_["cross_right"] = 22;
    button_map_["cross_up"] = 23;
    button_map_["cross_down"] = 24;
    axis_map_["left_stick_left_right"] = 0;
    axis_map_["left_stick_up_down"] = 1;
    axis_map_["lt"] = 2;
    axis_map_["right_stick_left_right"] = 3;
    axis_map_["right_stick_up_down"] = 4;
    axis_map_["rt"] = 5;
    axis_map_["cross_left_right"] = 6;
    axis_map_["cross_up_down"] = 7;
    reset();
  }

  void reset()
  {
    joy_msg_.axes = std::vector<float>( MAX_AXES, 0.0 );
    joy_msg_.buttons = std::vector<int>( MAX_BUTTONS, 0 );
    joy_msg_.axes[axis_map_["rt"]] = 1;
    joy_msg_.axes[axis_map_["lt"]] = 1;
    if ( timer_ && !timer_->is_canceled() )
      timer_->cancel();
  }

  void setButton( const std::string &button, const int value, const bool do_reset = false )
  {
    if ( do_reset )
      reset();
    joy_msg_.buttons[button_map_[button]] = value;
  }

  void setAxis( const std::string &axis, float value, const bool do_reset = false )
  {
    if ( do_reset )
      reset();
    if ( axis == "lt" || axis == "rt" )
      value = -value;
    joy_msg_.axes[axis_map_[axis]] = value;
  }

  void publishJoy() const { pub_->publish( joy_msg_ ); }

  void publishJoyContinuously()
  {
    timer_ = create_wall_timer( std::chrono::milliseconds( 33 ),
                                [this]() { pub_->publish( joy_msg_ ); } );
  }
};

template<class T>
class Subscriber
{
public:
  std::shared_ptr<rclcpp::Subscription<T>> sub_;
  T msg_;
  bool received_msg_ = false;

  Subscriber( rclcpp::Node::SharedPtr node, const std::string &topic, bool latched = false )
  {
    rclcpp::QoS qos_profile( rclcpp::KeepLast( 10 ) );
    if ( latched )
      qos_profile.durability( RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL );
    sub_ = node->create_subscription<T>( topic, qos_profile, [this]( const std::shared_ptr<T> msg ) {
      msg_ = *msg;
      received_msg_ = true;
    } );
  }

  void reset() { received_msg_ = false; }

  bool isConnected() { return sub_->get_publisher_count() > 0; }
};

class FakeReceiver final
{
public:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Subscriber<std_msgs::msg::String>> active_config_sub_;
  std::shared_ptr<Subscriber<geometry_msgs::msg::TwistStamped>> cmd_vel_sub_;
  std::shared_ptr<Subscriber<geometry_msgs::msg::TwistStamped>> twist_eef_sub_;
  std::shared_ptr<Subscriber<std_msgs::msg::Float64>> gripper_cmd_sub_;
  std::shared_ptr<Subscriber<std_msgs::msg::Float64MultiArray>> flipper_cmd_sub_;

  explicit FakeReceiver( const rclcpp::Node::SharedPtr &node ) : node_( node )
  {

    active_config_sub_ =
        std::make_shared<Subscriber<std_msgs::msg::String>>( node_, "/ocs/active_config", true );
    cmd_vel_sub_ =
        std::make_shared<Subscriber<geometry_msgs::msg::TwistStamped>>( node_, "/athena/cmd_vel" );
    twist_eef_sub_ = std::make_shared<Subscriber<geometry_msgs::msg::TwistStamped>>(
        node_, "/athena/moveit_twist_controller/eef_cmd" );
    gripper_cmd_sub_ = std::make_shared<Subscriber<std_msgs::msg::Float64>>(
        node_, "/athena/moveit_twist_controller/gripper_vel_cmd" );
    flipper_cmd_sub_ = std::make_shared<Subscriber<std_msgs::msg::Float64MultiArray>>(
        node_, "/athena/flipper_velocity_controller/commands" );
  }

  void reset() const
  {
    active_config_sub_->reset();
    cmd_vel_sub_->reset();
    twist_eef_sub_->reset();
    gripper_cmd_sub_->reset();
    flipper_cmd_sub_->reset();
  }

  bool isConnected() const
  {
    bool isConnected = true;
    isConnected &= active_config_sub_->isConnected();
    isConnected &= cmd_vel_sub_->isConnected();
    isConnected &= twist_eef_sub_->isConnected();
    isConnected &= gripper_cmd_sub_->isConnected();
    isConnected &= flipper_cmd_sub_->isConnected();
    return isConnected;
  }
};

class HectorGamepadManagerTest : public ::testing::Test
{
protected:
  std::shared_ptr<FakeJoyNode> fake_joy_node_;
  std::shared_ptr<FakeReceiver> fake_receiver_;
  rclcpp::Node::SharedPtr fake_receiver_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  void SetUp() override
  {
    rclcpp::init( 0, nullptr );

    // create the fake nodes
    fake_joy_node_ = std::make_shared<FakeJoyNode>();
    fake_receiver_node_ = std::make_shared<rclcpp::Node>( "FakeReceiverNode" );
    fake_receiver_ = std::make_shared<FakeReceiver>( fake_receiver_node_ );

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node( fake_joy_node_ );
    executor_->add_node( fake_receiver_node_ );
    RCLCPP_INFO( rclcpp::get_logger( "gamepad_manager" ), "Waiting for connection" );
    waitForConnection();
  }

  void TearDown() override { rclcpp::shutdown(); }

  // wait until publisher of fake_user_node_ has a subscriber
  void waitForConnection() const
  {
    rclcpp::Rate rate( 10 );
    while ( !fake_receiver_->isConnected() ) {
      executor_->spin_some();
      rate.sleep();
      print( "Waiting for connection" );
    }
    print( "Connected" );
  }

  // wait until fakes_user_node_ has received a twist message + timeout
  template<class T>
  bool waitForMsg( std::shared_ptr<Subscriber<T>> sub, double timeout_s )
  {
    constexpr double frequency = 10;
    rclcpp::Rate rate( frequency );
    double time = 0;
    while ( !sub->received_msg_ && time < timeout_s ) {
      executor_->spin_some();
      rate.sleep();
      time += 1 / frequency;
    }
    return sub->received_msg_;
  }

  void sleep( const double seconds ) const
  {
    constexpr double frequency = 20;
    rclcpp::Rate rate( frequency );
    for ( int i = 0; i < seconds * frequency; i++ ) {
      executor_->spin_some();
      rate.sleep();
    }
  }

  void makeSureConfigIsActive( const std::string &config )
  {
    if ( fake_receiver_->active_config_sub_->msg_.data != config ) {
      if ( config == "manipulation" )
        fake_joy_node_->setButton( "back", 1 );
      else if ( config == "driving" )
        fake_joy_node_->setButton( "start", 1 );
      else
        throw std::runtime_error( "unexpected config" );
      print( "Requesting Config Switch to " + config );
      fake_receiver_->reset();
      fake_joy_node_->publishJoy();
      // if already in config, no message will be published
      waitForMsg( fake_receiver_->active_config_sub_, 0.5 );
      fake_joy_node_->reset(); // undo button press
    }
  }

  void print( const std::string &msg ) const
  {
    RCLCPP_INFO( fake_receiver_node_->get_logger(), msg.c_str() );
  }
};

// test that config switching is published
TEST_F( HectorGamepadManagerTest, SwitchConfig )
{
  // driving set as default config and should be published
  ASSERT_TRUE( waitForMsg( fake_receiver_->active_config_sub_, TIMEOUT ) );
  EXPECT_EQ( fake_receiver_->active_config_sub_->msg_.data, "driving" );

  // switch to manipulation config -> back button
  fake_receiver_->reset();
  fake_joy_node_->setButton( "back", 1 );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->active_config_sub_, TIMEOUT ) );
  EXPECT_EQ( fake_receiver_->active_config_sub_->msg_.data, "manipulation" );

  // switch back to driving config -> start button
  fake_receiver_->reset();
  fake_joy_node_->reset();
  fake_joy_node_->setButton( "start", 1 );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->active_config_sub_, TIMEOUT ) );
  EXPECT_EQ( fake_receiver_->active_config_sub_->msg_.data, "driving" );

  // if switching to current config,
  fake_receiver_->reset();
  fake_joy_node_->publishJoy(); // driving button still pressed
  ASSERT_FALSE( waitForMsg( fake_receiver_->active_config_sub_, TIMEOUT ) );
}

// test cmd_vel
TEST_F( HectorGamepadManagerTest, CmdVel )
{
  // driving config
  makeSureConfigIsActive( "driving" );
  fake_joy_node_->setAxis( "left_stick_left_right", 0.5 );
  fake_joy_node_->setAxis( "left_stick_up_down", 0.5 );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->cmd_vel_sub_, TIMEOUT ) );
  EXPECT_GE( fake_receiver_->cmd_vel_sub_->msg_.twist.linear.x, 0.0 );
  EXPECT_EQ( fake_receiver_->cmd_vel_sub_->msg_.twist.linear.y, 0.0 );
  EXPECT_EQ( fake_receiver_->cmd_vel_sub_->msg_.twist.linear.z, 0.0 );
  EXPECT_EQ( fake_receiver_->cmd_vel_sub_->msg_.twist.angular.x, 0.0 );
  EXPECT_EQ( fake_receiver_->cmd_vel_sub_->msg_.twist.angular.y, 0.0 );
  EXPECT_NE( fake_receiver_->cmd_vel_sub_->msg_.twist.angular.z, 0.0 );
  fake_receiver_->reset();
  fake_joy_node_->reset();
  fake_joy_node_->setAxis( "left_stick_up_down", -0.5 );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->cmd_vel_sub_, TIMEOUT ) );
  EXPECT_LT( fake_receiver_->cmd_vel_sub_->msg_.twist.linear.x, 0.0 );
  EXPECT_EQ( fake_receiver_->cmd_vel_sub_->msg_.twist.linear.y, 0.0 );
  EXPECT_EQ( fake_receiver_->cmd_vel_sub_->msg_.twist.linear.z, 0.0 );
  EXPECT_EQ( fake_receiver_->cmd_vel_sub_->msg_.twist.angular.x, 0.0 );
  EXPECT_EQ( fake_receiver_->cmd_vel_sub_->msg_.twist.angular.y, 0.0 );
  EXPECT_EQ( fake_receiver_->cmd_vel_sub_->msg_.twist.angular.z, 0.0 );
}

// test speed buttons
TEST_F( HectorGamepadManagerTest, SpeedButtons )
{
  // button 'a' -> fast -> 2.0 (see config in test folder)
  // button 'x' -> slow -> 0.3
  // normal speed is 0.6
  constexpr double max_linear_speed = 1.1;
  constexpr double max_angular_speed = 1.5;
  constexpr double normal_factor = 1.0;
  constexpr double fast_factor = 2.0;
  constexpr double slow_factor = 0.3;

  // test normal
  fake_joy_node_->setAxis( "left_stick_left_right", 1.0 );
  fake_joy_node_->setAxis( "left_stick_up_down", 1.0 );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->cmd_vel_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->cmd_vel_sub_->msg_.twist.linear.x, normal_factor * max_linear_speed,
               0.1 );
  EXPECT_NEAR( fake_receiver_->cmd_vel_sub_->msg_.twist.angular.z,
               normal_factor * max_angular_speed, 0.1 );

  // test slow
  fake_receiver_->reset();
  fake_joy_node_->setButton( "x", 1 );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->cmd_vel_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->cmd_vel_sub_->msg_.twist.linear.x, slow_factor * max_linear_speed,
               0.1 );
  EXPECT_NEAR( fake_receiver_->cmd_vel_sub_->msg_.twist.angular.z, slow_factor * max_angular_speed,
               0.1 );

  // test fast
  fake_receiver_->reset();
  fake_joy_node_->setButton( "a", 1 );
  fake_joy_node_->setButton( "x", 0 );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->cmd_vel_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->cmd_vel_sub_->msg_.twist.linear.x, fast_factor * max_linear_speed,
               0.1 );
  EXPECT_NEAR( fake_receiver_->cmd_vel_sub_->msg_.twist.angular.z, fast_factor * max_angular_speed,
               0.1 );
}

// make sure no more cmd sent when switching to manipulation config
// make sure zero twist is sent if switch to manipulation config
TEST_F( HectorGamepadManagerTest, NoMoreCmdVelTwistWhenSwitchingToManipulationConfig )
{
  makeSureConfigIsActive( "driving" );
  fake_joy_node_->setAxis( "left_stick_left_right", 1.0 );
  fake_joy_node_->setAxis( "left_stick_up_down", 1.0 );
  fake_receiver_->reset();
  fake_joy_node_->publishJoyContinuously();
  ASSERT_TRUE( waitForMsg( fake_receiver_->cmd_vel_sub_, TIMEOUT ) );

  // switch Config to manipulation -> back button
  fake_joy_node_->setButton( "back", 1 );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->active_config_sub_, TIMEOUT ) );
  EXPECT_EQ( fake_receiver_->active_config_sub_->msg_.data, "manipulation" );
  // make sure last received twist is zero
  EXPECT_EQ( fake_receiver_->cmd_vel_sub_->msg_.twist.linear.x, 0.0 );
  EXPECT_EQ( fake_receiver_->cmd_vel_sub_->msg_.twist.angular.z, 0.0 );

  // make sure no more cmd_vel twist is received
  fake_receiver_->reset();
  EXPECT_FALSE( waitForMsg( fake_receiver_->cmd_vel_sub_, TIMEOUT ) );
}

// test eef twist
TEST_F( HectorGamepadManagerTest, EefTwistLinear )
{
  makeSureConfigIsActive( "manipulation" );
  constexpr double max_linear_speed = 0.15;
  // positive y
  fake_joy_node_->setAxis( "left_stick_left_right", 1.0, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->twist_eef_sub_->msg_.twist.linear.y, max_linear_speed, 0.01 );

  // positive z
  fake_receiver_->reset();
  fake_joy_node_->setAxis( "left_stick_up_down", 1.0, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->twist_eef_sub_->msg_.twist.linear.z, max_linear_speed, 0.01 );

  // positive x
  fake_receiver_->reset();
  fake_joy_node_->setAxis( "rt", 1.0, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) ); // TODO
  EXPECT_NEAR( fake_receiver_->twist_eef_sub_->msg_.twist.linear.x, max_linear_speed, 0.01 );

  // negative y
  fake_receiver_->reset();
  fake_joy_node_->setAxis( "left_stick_left_right", -1.0, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->twist_eef_sub_->msg_.twist.linear.y, -max_linear_speed, 0.01 );

  // negative z
  fake_receiver_->reset();
  fake_joy_node_->setAxis( "left_stick_up_down", -1.0, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->twist_eef_sub_->msg_.twist.linear.z, -max_linear_speed, 0.01 );

  // negative x
  fake_receiver_->reset();
  fake_joy_node_->setAxis( "lt", 1.0, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->twist_eef_sub_->msg_.twist.linear.x, -max_linear_speed, 0.01 );
}

// test eef twist
TEST_F( HectorGamepadManagerTest, EefTwistAngular )
{
  makeSureConfigIsActive( "manipulation" );
  constexpr double max_angular_speed = 0.25;
  // positive yaw
  fake_joy_node_->setAxis( "right_stick_left_right", 1.0, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->twist_eef_sub_->msg_.twist.angular.z, max_angular_speed, 0.01 );

  // positive pitch
  fake_receiver_->reset();
  fake_joy_node_->setAxis( "right_stick_up_down", 1.0, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->twist_eef_sub_->msg_.twist.angular.y, max_angular_speed, 0.01 );

  // positive roll
  fake_receiver_->reset();
  fake_joy_node_->setButton( "rb", 1, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->twist_eef_sub_->msg_.twist.angular.x, max_angular_speed, 0.01 );

  // negative y
  fake_receiver_->reset();
  fake_joy_node_->setAxis( "right_stick_left_right", -1.0, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->twist_eef_sub_->msg_.twist.angular.z, -max_angular_speed, 0.01 );

  // negative z
  fake_receiver_->reset();
  fake_joy_node_->setAxis( "right_stick_up_down", -1.0, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->twist_eef_sub_->msg_.twist.angular.y, -max_angular_speed, 0.01 );

  // negative x
  fake_receiver_->reset();
  fake_joy_node_->setButton( "lb", 1, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NEAR( fake_receiver_->twist_eef_sub_->msg_.twist.angular.x, -max_angular_speed, 0.01 );
}

// test that zero eef twist sent if switching to driving config
TEST_F( HectorGamepadManagerTest, SendZeroEefTwistWhenActivatingDriving )
{
  makeSureConfigIsActive( "manipulation" );
  fake_joy_node_->setAxis( "right_stick_left_right", 1.0 );
  fake_receiver_->reset();
  fake_joy_node_->publishJoyContinuously();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NE( fake_receiver_->twist_eef_sub_->msg_.twist.angular.z, 0.0 );

  // switch Config to driving -> start button
  fake_joy_node_->setButton( "start", 1 );
  fake_receiver_->reset();
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->active_config_sub_, TIMEOUT ) );
  EXPECT_EQ( fake_receiver_->active_config_sub_->msg_.data, "driving" );
  // make sure last received eef twist is zero
  EXPECT_EQ( fake_receiver_->twist_eef_sub_->msg_.twist.linear.x, 0.0 );
  EXPECT_EQ( fake_receiver_->twist_eef_sub_->msg_.twist.linear.y, 0.0 );
  EXPECT_EQ( fake_receiver_->twist_eef_sub_->msg_.twist.linear.z, 0.0 );
  EXPECT_EQ( fake_receiver_->twist_eef_sub_->msg_.twist.angular.x, 0.0 );
  EXPECT_EQ( fake_receiver_->twist_eef_sub_->msg_.twist.angular.y, 0.0 );
  EXPECT_EQ( fake_receiver_->twist_eef_sub_->msg_.twist.angular.z, 0.0 );
}

TEST_F( HectorGamepadManagerTest, GripperFunctions )
{
  makeSureConfigIsActive( "manipulation" );
  constexpr double gripper_speed = 0.2; // see config in test folder
  // open gripper -> button 'y'
  fake_joy_node_->setButton( "y", 1, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->gripper_cmd_sub_, TIMEOUT ) );
  EXPECT_EQ( fake_receiver_->gripper_cmd_sub_->msg_.data, gripper_speed );

  // close gripper -> button 'x'
  fake_receiver_->reset();
  fake_joy_node_->reset();
  fake_joy_node_->setButton( "x", 1, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->gripper_cmd_sub_, TIMEOUT ) );
  EXPECT_EQ( fake_receiver_->gripper_cmd_sub_->msg_.data, -gripper_speed );
}

TEST_F( HectorGamepadManagerTest, ZeroGripperWhenSwitchingToDriving )
{
  makeSureConfigIsActive( "manipulation" );
  constexpr double gripper_speed = 0.2; // see config in test folder
  // open gripper -> button 'y'
  fake_joy_node_->setButton( "y", 1, true );
  fake_joy_node_->publishJoyContinuously();
  ASSERT_TRUE( waitForMsg( fake_receiver_->gripper_cmd_sub_, TIMEOUT ) );
  EXPECT_EQ( fake_receiver_->gripper_cmd_sub_->msg_.data, gripper_speed );

  // switch Config to driving -> start button
  fake_joy_node_->setButton( "start", 1 );
  fake_receiver_->reset();
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->active_config_sub_, TIMEOUT ) );
  EXPECT_EQ( fake_receiver_->active_config_sub_->msg_.data, "driving" );

  // make sure last received gripper cmd is zero
  EXPECT_EQ( fake_receiver_->gripper_cmd_sub_->msg_.data, 0.0 );
}

TEST_F( HectorGamepadManagerTest, HoldModeDisablesEefCmds )
{
  makeSureConfigIsActive( "manipulation" );
  // sent continuous twist with left joystick
  fake_joy_node_->setAxis( "left_stick_up_down", 1.0, true );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  EXPECT_NE( fake_receiver_->twist_eef_sub_->msg_.twist.linear.z, 0.0 );

  // activate hold mode -> button 'b'
  fake_joy_node_->setButton( "b", 1 );
  fake_receiver_->reset();
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->twist_eef_sub_, TIMEOUT ) );
  // make sure last received twist is zero
  EXPECT_EQ( fake_receiver_->twist_eef_sub_->msg_.twist.linear.z, 0.0 );

  // make sure cmd vel commands are sent in hold mode
  fake_receiver_->reset();
  fake_joy_node_->publishJoy();
  EXPECT_TRUE( waitForMsg( fake_receiver_->cmd_vel_sub_, TIMEOUT ) );
  // since hold mode is active and left joystick used, expect cmd vel twist
  EXPECT_NE( fake_receiver_->cmd_vel_sub_->msg_.twist.linear.x, 0.0 );
  EXPECT_EQ( fake_receiver_->cmd_vel_sub_->msg_.twist.angular.z, 0.0 );
}

// Flipper tests
TEST_F( HectorGamepadManagerTest, FlipperCmdsReceived )
{
  constexpr double flipper_speed = 1.5;
  makeSureConfigIsActive( "driving" );
  // flipper up -> buttons lb and rb
  fake_joy_node_->setButton( "lb", 1, true );
  fake_joy_node_->setButton( "rb", 1, false );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->flipper_cmd_sub_, TIMEOUT ) );
  ASSERT_EQ( fake_receiver_->flipper_cmd_sub_->msg_.data.size(), 4 );
  for ( size_t i = 0; i < 4; ++i )
    EXPECT_EQ( fake_receiver_->flipper_cmd_sub_->msg_.data[i], flipper_speed );

  // flipper down -> axis rt and lt
  fake_receiver_->reset();
  fake_joy_node_->setAxis( "rt", 1.0, true );
  fake_joy_node_->setAxis( "lt", 1.0, false );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->flipper_cmd_sub_, TIMEOUT ) );
  ASSERT_EQ( fake_receiver_->flipper_cmd_sub_->msg_.data.size(), 4 );
  for ( size_t i = 0; i < 4; ++i )
    EXPECT_EQ( fake_receiver_->flipper_cmd_sub_->msg_.data[i], -flipper_speed );
}

/*TEST_F( HectorGamepadManagerTest, FlipperIndividualCmdsReceived )
{
  constexpr double flipper_speed = 1.5;
  makeSureConfigIsActive( "driving" );
  constexpr int flipper_front_left = 0;
  constexpr int flipper_front_right = 1;
  constexpr int flipper_back_left = 2;
  constexpr int flipper_back_right = 3;
  // flipper front left up, right down ->
  fake_joy_node_->reset();
  fake_joy_node_->publishJoy(); // flipper plugin requires explicit release of buttons
  ASSERT_TRUE( waitForMsg( fake_receiver_->flipper_cmd_sub_, TIMEOUT ) );
  fake_receiver_->reset();
  fake_joy_node_->setButton( "y", 1, true ); // select individual front flipper mode
  fake_joy_node_->setButton( "rb", 1, false );
  fake_joy_node_->setButton( "lb", 1, false );
  //fake_joy_node_->setAxis( "lt", 1.0, false );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->flipper_cmd_sub_, TIMEOUT ) );
  fake_receiver_->reset();
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->flipper_cmd_sub_, TIMEOUT ) );

  std::stringstream ss;
  ss << "flipper velocities: \n";
  for ( size_t i = 0; i < 4; ++i ) ss << fake_receiver_->flipper_cmd_sub_->msg_.data[i] << "\n";
  print( ss.str() );
  ASSERT_EQ(  fake_receiver_->flipper_cmd_sub_->msg_.data.size(), 4 );
  EXPECT_EQ( fake_receiver_->flipper_cmd_sub_->msg_.data[flipper_front_left], flipper_speed );
  EXPECT_EQ( fake_receiver_->flipper_cmd_sub_->msg_.data[flipper_front_right], -flipper_speed );
  EXPECT_EQ( fake_receiver_->flipper_cmd_sub_->msg_.data[flipper_back_left], 0.0 );
  EXPECT_EQ( fake_receiver_->flipper_cmd_sub_->msg_.data[flipper_back_right], 0.0 );

  // flipper down -> axis rt and lt
  fake_receiver_->reset();
  fake_joy_node_->setAxis( "rt", 1.0, true );
  fake_joy_node_->setAxis( "lt", 1.0, false );
  fake_joy_node_->publishJoy();
  ASSERT_TRUE( waitForMsg( fake_receiver_->flipper_cmd_sub_, TIMEOUT ) );
  EXPECT_EQ( fake_receiver_->flipper_cmd_sub_->msg_.data.size(), 4 );
  for ( size_t i = 0; i < 4; ++i )
    EXPECT_EQ( fake_receiver_->flipper_cmd_sub_->msg_.data[i], -flipper_speed );
}*/
