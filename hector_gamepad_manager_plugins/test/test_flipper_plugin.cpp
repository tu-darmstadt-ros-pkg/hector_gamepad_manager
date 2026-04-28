#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>
#include <rtest/action_client_mock.hpp>

#include <hector_gamepad_manager_plugins/flipper_plugin.hpp>
#include <hector_gamepad_plugin_interface/blackboard.hpp>
#include <hector_gamepad_plugin_interface/feedback_manager.hpp>

#include <hector_ros_controllers_msgs/action/drive_flipper_group.hpp>
#include <hector_ros_controllers_msgs/action/sync_flipper_group.hpp>

using ::testing::_;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::SaveArg;

using DriveAction = hector_ros_controllers_msgs::action::DriveFlipperGroup;
using SyncAction = hector_ros_controllers_msgs::action::SyncFlipperGroup;

// Direct unit tests for FlipperPlugin's action-client paths; bypasses pluginlib to attach action_client_mock without pulling in BatteryMonitorPlugin's babel_fish dependency.
class FlipperPluginTest : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<hector_gamepad_manager_plugins::FlipperPlugin> plugin_;
  std::shared_ptr<rtest::experimental::ActionClientMock<DriveAction>> drive_mock_;
  std::shared_ptr<rtest::experimental::ActionClientMock<SyncAction>> sync_mock_;

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>( "flipper_plugin_test" );
    node_->declare_parameter<std::string>( "robot_namespace", "athena" );

    plugin_ = std::make_shared<hector_gamepad_manager_plugins::FlipperPlugin>();
    plugin_->initializePlugin( node_, node_, "hector_gamepad_manager_plugins::FlipperPlugin",
                               std::make_shared<hector_gamepad_plugin_interface::Blackboard>(),
                               std::make_shared<hector_gamepad_plugin_interface::FeedbackManager>(),
                               nullptr );

    drive_mock_ = rtest::experimental::findActionClient<DriveAction>(
        node_, "/athena/vel_to_pos_controller/drive_flipper_group" );
    sync_mock_ = rtest::experimental::findActionClient<SyncAction>(
        node_, "/athena/vel_to_pos_controller/sync_flipper_group" );

    ASSERT_TRUE( drive_mock_ );
    ASSERT_TRUE( sync_mock_ );
  }
};

// flipper_front_upright sends a goal with group_name="flipper_front" and target_position 0.
TEST_F( FlipperPluginTest, FrontUprightSendsGoalWithFrontGroup )
{
  EXPECT_CALL( *drive_mock_, action_server_is_ready() ).WillRepeatedly( Return( true ) );

  DriveAction::Goal sent_goal{};
  EXPECT_CALL( *drive_mock_, async_send_goal( _, _ ) )
      .WillOnce( DoAll( SaveArg<0>( &sent_goal ),
                        rtest::experimental::ReturnGoalHandleFuture(
                            rclcpp_action::ClientGoalHandle<DriveAction>::SharedPtr{} ) ) );

  plugin_->handlePress( "flipper_front_upright", "test_id" );

  EXPECT_EQ( sent_goal.group_name, "flipper_front" );
  EXPECT_EQ( sent_goal.target_position, 0.0 );
}

// flipper_back_upright routes to group_name="flipper_back" on the same client.
TEST_F( FlipperPluginTest, BackUprightSendsGoalWithBackGroup )
{
  EXPECT_CALL( *drive_mock_, action_server_is_ready() ).WillRepeatedly( Return( true ) );

  DriveAction::Goal sent_goal{};
  EXPECT_CALL( *drive_mock_, async_send_goal( _, _ ) )
      .WillOnce( DoAll( SaveArg<0>( &sent_goal ),
                        rtest::experimental::ReturnGoalHandleFuture(
                            rclcpp_action::ClientGoalHandle<DriveAction>::SharedPtr{} ) ) );

  plugin_->handlePress( "flipper_back_upright", "test_id" );

  EXPECT_EQ( sent_goal.group_name, "flipper_back" );
}

// When the action server isn't ready, no goal is sent — the plugin only logs a warning.
TEST_F( FlipperPluginTest, UprightSkippedWhenServerNotReady )
{
  EXPECT_CALL( *drive_mock_, action_server_is_ready() ).WillRepeatedly( Return( false ) );
  EXPECT_CALL( *drive_mock_, async_send_goal( _, _ ) ).Times( 0 );

  plugin_->handlePress( "flipper_front_upright", "test_id" );
}

// sync_front_flippers sends a SyncFlipperGroup goal with group_names=["flipper_front"].
TEST_F( FlipperPluginTest, SyncFrontSendsGoalWithFrontGroup )
{
  EXPECT_CALL( *sync_mock_, action_server_is_ready() ).WillRepeatedly( Return( true ) );

  SyncAction::Goal sent_goal{};
  EXPECT_CALL( *sync_mock_, async_send_goal( _, _ ) )
      .WillOnce( DoAll( SaveArg<0>( &sent_goal ),
                        rtest::experimental::ReturnGoalHandleFuture(
                            rclcpp_action::ClientGoalHandle<SyncAction>::SharedPtr{} ) ) );

  plugin_->handlePress( "sync_front_flippers", "test_id" );

  ASSERT_EQ( sent_goal.group_names.size(), 1u );
  EXPECT_EQ( sent_goal.group_names[0], "flipper_front" );
}

// sync_all_flippers sends an empty group_names list — the contract for "all groups".
TEST_F( FlipperPluginTest, SyncAllSendsGoalWithEmptyGroupList )
{
  EXPECT_CALL( *sync_mock_, action_server_is_ready() ).WillRepeatedly( Return( true ) );

  SyncAction::Goal sent_goal{};
  EXPECT_CALL( *sync_mock_, async_send_goal( _, _ ) )
      .WillOnce( DoAll( SaveArg<0>( &sent_goal ),
                        rtest::experimental::ReturnGoalHandleFuture(
                            rclcpp_action::ClientGoalHandle<SyncAction>::SharedPtr{} ) ) );

  plugin_->handlePress( "sync_all_flippers", "test_id" );

  EXPECT_TRUE( sent_goal.group_names.empty() );
}

// Regression: a SUCCEEDED result with a null result pointer must not crash the callback.
TEST_F( FlipperPluginTest, UprightResultCallbackHandlesNullResultPayload )
{
  EXPECT_CALL( *drive_mock_, action_server_is_ready() ).WillRepeatedly( Return( true ) );

  rclcpp_action::Client<DriveAction>::SendGoalOptions captured_opts;
  auto goal_handle = drive_mock_->makeClientGoalHandle();
  EXPECT_CALL( *drive_mock_, async_send_goal( _, _ ) )
      .WillOnce( DoAll( SaveArg<1>( &captured_opts ),
                        rtest::experimental::ReturnGoalHandleFuture( goal_handle ) ) );

  plugin_->handlePress( "flipper_front_upright", "test_id" );

  ASSERT_NE( captured_opts.result_callback, nullptr );

  rclcpp_action::ClientGoalHandle<DriveAction>::WrappedResult result;
  result.code = rclcpp_action::ResultCode::SUCCEEDED;
  result.result = nullptr; // the case the recent fix guards against
  EXPECT_NO_THROW( captured_opts.result_callback( result ) );
}

// Same regression check for the sync action's result callback.
TEST_F( FlipperPluginTest, SyncResultCallbackHandlesNullResultPayload )
{
  EXPECT_CALL( *sync_mock_, action_server_is_ready() ).WillRepeatedly( Return( true ) );

  rclcpp_action::Client<SyncAction>::SendGoalOptions captured_opts;
  auto goal_handle = sync_mock_->makeClientGoalHandle();
  EXPECT_CALL( *sync_mock_, async_send_goal( _, _ ) )
      .WillOnce( DoAll( SaveArg<1>( &captured_opts ),
                        rtest::experimental::ReturnGoalHandleFuture( goal_handle ) ) );

  plugin_->handlePress( "sync_back_flippers", "test_id" );

  ASSERT_NE( captured_opts.result_callback, nullptr );

  rclcpp_action::ClientGoalHandle<SyncAction>::WrappedResult result;
  result.code = rclcpp_action::ResultCode::ABORTED;
  result.result = nullptr;
  EXPECT_NO_THROW( captured_opts.result_callback( result ) );
}

// A second upright press while a goal is in flight must be dropped; clears after result.
TEST_F( FlipperPluginTest, UprightDropsRequestWhileGoalInFlight )
{
  EXPECT_CALL( *drive_mock_, action_server_is_ready() ).WillRepeatedly( Return( true ) );

  rclcpp_action::Client<DriveAction>::SendGoalOptions captured_opts;
  auto goal_handle = drive_mock_->makeClientGoalHandle();
  EXPECT_CALL( *drive_mock_, async_send_goal( _, _ ) )
      .WillOnce( DoAll( SaveArg<1>( &captured_opts ),
                        rtest::experimental::ReturnGoalHandleFuture( goal_handle ) ) );

  plugin_->handlePress( "flipper_front_upright", "test_id" );
  ASSERT_NE( captured_opts.goal_response_callback, nullptr );
  captured_opts.goal_response_callback( goal_handle );

  EXPECT_CALL( *drive_mock_, async_send_goal( _, _ ) ).Times( 0 );
  plugin_->handlePress( "flipper_front_upright", "test_id" );

  ASSERT_NE( captured_opts.result_callback, nullptr );
  rclcpp_action::ClientGoalHandle<DriveAction>::WrappedResult result;
  result.code = rclcpp_action::ResultCode::SUCCEEDED;
  result.result = nullptr;
  captured_opts.result_callback( result );

  EXPECT_CALL( *drive_mock_, async_send_goal( _, _ ) )
      .WillOnce( rtest::experimental::ReturnGoalHandleFuture(
          rclcpp_action::ClientGoalHandle<DriveAction>::SharedPtr{} ) );
  plugin_->handlePress( "flipper_front_upright", "test_id" );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleMock( &argc, argv );
  rclcpp::init( argc, argv );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
