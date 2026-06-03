#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>
#include <rtest/publisher_mock.hpp>
#include <rtest/service_mock.hpp>
#include <rtest/subscription_mock.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

#include <hector_gamepad_manager/joy_satellite.hpp>

using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Each;
using ::testing::Field;
using ::testing::SaveArg;

using SetTargetRobot = hector_gamepad_manager_msgs::srv::SetTargetRobot;

// The satellite node is constructed at the root namespace, so its relative topics resolve to
// "/joy" (input) and "/joy/set_feedback" (rumble out); the robot-facing topics are absolute. It
// starts targeting "athena" so the robot-facing endpoints exist from construction.
class JoySatelliteTest : public ::testing::Test
{
protected:
  std::shared_ptr<hector_gamepad_manager::JoySatelliteNode> node_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> sub_joy_;
  std::shared_ptr<rtest::PublisherMock<sensor_msgs::msg::Joy>> pub_athena_;
  std::shared_ptr<rtest::PublisherMock<sensor_msgs::msg::JoyFeedback>> pub_feedback_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JoyFeedback>> sub_feedback_athena_;
  std::shared_ptr<rtest::ServiceMock<SetTargetRobot>> srv_;

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    // Absolute target (leading '/'): the robot-facing topics resolve to /athena/joy regardless of
    // the satellite's own namespace. A relative target (e.g. "athena") is also accepted and would
    // resolve under the node's namespace instead - see SwitchRetargetsAndNeutralizesOldRobot.
    options.parameter_overrides( { rclcpp::Parameter( "target_robot", "/athena" ) } );
    node_ = std::make_shared<hector_gamepad_manager::JoySatelliteNode>( options );

    sub_joy_ = rtest::findSubscription<sensor_msgs::msg::Joy>( node_, "/joy" );
    pub_athena_ = rtest::findPublisher<sensor_msgs::msg::Joy>( node_, "/athena/joy" );
    pub_feedback_ =
        rtest::findPublisher<sensor_msgs::msg::JoyFeedback>( node_, "/joy/set_feedback" );
    sub_feedback_athena_ =
        rtest::findSubscription<sensor_msgs::msg::JoyFeedback>( node_, "/athena/joy_feedback" );
    srv_ = rtest::findService<SetTargetRobot>( node_, "set_target_robot" );

    ASSERT_TRUE( sub_joy_ );
    ASSERT_TRUE( pub_athena_ );
    ASSERT_TRUE( pub_feedback_ );
    ASSERT_TRUE( sub_feedback_athena_ );
    ASSERT_TRUE( srv_ );
  }

  static sensor_msgs::msg::Joy makeJoy()
  {
    sensor_msgs::msg::Joy joy;
    joy.axes = { 0.5f, -0.5f };
    joy.buttons = { 1, 0, 1 };
    return joy;
  }

  // Drive the set_target_robot service and return the captured response.
  SetTargetRobot::Response callSetTarget( const std::string &ns )
  {
    auto request = std::make_shared<SetTargetRobot::Request>();
    request->robot_namespace = ns;
    SetTargetRobot::Response captured;
    EXPECT_CALL( *srv_, send_response( _, _ ) ).WillOnce( SaveArg<1>( &captured ) );
    srv_->handle_request( std::make_shared<rmw_request_id_t>(), request );
    return captured;
  }
};

// Incoming joy is republished to the currently selected robot's joy topic.
TEST_F( JoySatelliteTest, ForwardsJoyToTarget )
{
  auto joy = makeJoy();
  EXPECT_CALL( *pub_athena_, publish( Field( &sensor_msgs::msg::Joy::buttons, joy.buttons ) ) )
      .Times( 1 );
  sub_joy_->handle_message( joy );
}

// The selected robot's rumble feedback is bridged back to the local feedback topic.
TEST_F( JoySatelliteTest, BridgesFeedbackBack )
{
  sensor_msgs::msg::JoyFeedback fb;
  fb.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
  fb.intensity = 0.7f;
  EXPECT_CALL( *pub_feedback_,
               publish( Field( &sensor_msgs::msg::JoyFeedback::intensity, 0.7f ) ) )
      .Times( 1 );
  sub_feedback_athena_->handle_message( fb );
}

// Switching robots sends one neutral (all-zero) joy to the old robot and routes new joy to the new.
TEST_F( JoySatelliteTest, SwitchRetargetsAndNeutralizesOldRobot )
{
  // Forward a joy first so the neutral on switch matches the gamepad's axis/button layout.
  auto joy = makeJoy();
  EXPECT_CALL( *pub_athena_, publish( Field( &sensor_msgs::msg::Joy::buttons, joy.buttons ) ) )
      .Times( 1 );
  sub_joy_->handle_message( joy );

  // Switching neutralizes the old robot exactly once (all-zero, matching the forwarded layout).
  EXPECT_CALL( *pub_athena_,
               publish( AllOf( Field( &sensor_msgs::msg::Joy::axes, Each( 0.0f ) ),
                               Field( &sensor_msgs::msg::Joy::buttons, Each( 0 ) ) ) ) )
      .Times( 1 );

  const auto response = callSetTarget( "bob" );
  EXPECT_TRUE( response.success );

  // New joy is now routed to the new robot.
  auto pub_bob = rtest::findPublisher<sensor_msgs::msg::Joy>( node_, "/bob/joy" );
  ASSERT_TRUE( pub_bob );
  EXPECT_CALL( *pub_bob, publish( Field( &sensor_msgs::msg::Joy::buttons, joy.buttons ) ) )
      .Times( 1 );
  sub_joy_->handle_message( joy );
}

// An empty robot namespace detaches: the previously selected robot is neutralized and forwarding
// stops until a new target is set.
TEST_F( JoySatelliteTest, EmptyNamespaceDetaches )
{
  // Forward a joy first so the detach neutral matches the gamepad layout (and forwarding works).
  auto joy = makeJoy();
  EXPECT_CALL( *pub_athena_, publish( Field( &sensor_msgs::msg::Joy::buttons, joy.buttons ) ) )
      .Times( 1 );
  sub_joy_->handle_message( joy );

  // Detaching succeeds and neutralizes the previously selected robot exactly once.
  EXPECT_CALL( *pub_athena_,
               publish( AllOf( Field( &sensor_msgs::msg::Joy::axes, Each( 0.0f ) ),
                               Field( &sensor_msgs::msg::Joy::buttons, Each( 0 ) ) ) ) )
      .Times( 1 );
  const auto response = callSetTarget( "" );
  EXPECT_TRUE( response.success );

  // With no target selected, further joy is dropped (and must not crash).
  sub_joy_->handle_message( joy );
}

// A malformed namespace (would yield an invalid topic name) is rejected without crashing the node,
// and the current target is kept. Note an absolute namespace like "/athena" is valid (it yields the
// valid topic "/athena/joy"); only names that produce an invalid topic are rejected.
TEST_F( JoySatelliteTest, RejectsMalformedNamespace )
{
  EXPECT_CALL( *pub_athena_, publish( _ ) ).Times( AnyNumber() );
  for ( const std::string bad : { "athena/", "robot ns", "1robot", "a//b", "//bob" } ) {
    const auto response = callSetTarget( bad );
    EXPECT_FALSE( response.success ) << "namespace '" << bad << "' should be rejected";
  }

  // Still forwarding to the original target.
  auto joy = makeJoy();
  EXPECT_CALL( *pub_athena_, publish( Field( &sensor_msgs::msg::Joy::buttons, joy.buttons ) ) )
      .Times( 1 );
  sub_joy_->handle_message( joy );
}
