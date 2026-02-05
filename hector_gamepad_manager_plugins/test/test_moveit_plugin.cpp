#include <gtest/gtest.h>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#define private public
#define protected public
#include <hector_gamepad_manager_plugins/moveit_plugin.hpp>
#undef protected
#undef private

#include <controller_orchestrator/controller_orchestrator.hpp>
#include <hector_gamepad_plugin_interface/blackboard.hpp>
#include <hector_gamepad_plugin_interface/feedback_manager.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>

using namespace std::chrono_literals;

namespace
{
constexpr const char *kPluginId = "hector_gamepad_manager_plugins::MoveitPlugin";
constexpr const char *kGroup = "arm";
constexpr const char *kMoveActionTopic = "/move_action";
constexpr const char *kTrajectoryActionTopic = "/trajectory_controller/follow_joint_trajectory";
constexpr const char *kViaPose = "via";
constexpr const char *kTargetPose = "target";
constexpr const char *kHomePose = "home";
constexpr const char *kParkedPose = "parked";
constexpr const char *kTargetId = "target_pose";
constexpr const char *kParkedId = "parked_pose";

const char kUrdf[] = R"(
<robot name="test_robot">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="joint2" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="1" velocity="1"/>
  </joint>
</robot>
)";

const char kSrdf[] = R"(
<robot name="test_robot">
  <group name="arm">
    <joint name="joint1"/>
    <joint name="joint2"/>
  </group>
  <group_state name="home" group="arm">
    <joint name="joint1" value="0.0"/>
    <joint name="joint2" value="0.0"/>
  </group_state>
  <group_state name="via" group="arm">
    <joint name="joint1" value="0.5"/>
    <joint name="joint2" value="0.5"/>
  </group_state>
  <group_state name="target" group="arm">
    <joint name="joint1" value="1.0"/>
    <joint name="joint2" value="1.0"/>
  </group_state>
  <group_state name="parked" group="arm">
    <joint name="joint1" value="-0.5"/>
    <joint name="joint2" value="-0.5"/>
  </group_state>
</robot>
)";

std::unordered_map<std::string, double>
constraintsToMap( const std::vector<moveit_msgs::msg::JointConstraint> &constraints )
{
  std::unordered_map<std::string, double> result;
  for ( const auto &constraint : constraints ) {
    result[constraint.joint_name] = constraint.position;
  }
  return result;
}

std::unordered_map<std::string, double>
trajectoryToMap( const trajectory_msgs::msg::JointTrajectory &trajectory )
{
  std::unordered_map<std::string, double> result;
  if ( trajectory.points.empty() ) {
    return result;
  }
  const auto &point = trajectory.points.front();
  for ( size_t i = 0; i < trajectory.joint_names.size() && i < point.positions.size(); ++i ) {
    result[trajectory.joint_names[i]] = point.positions[i];
  }
  return result;
}

} // namespace

class MoveitPluginTest : public hector_testing_utils::HectorTestFixture
{
protected:
  std::shared_ptr<hector_gamepad_plugin_interface::Blackboard> blackboard_;
  std::shared_ptr<hector_gamepad_plugin_interface::FeedbackManager> feedback_manager_;
  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> controller_orchestrator_;
  std::shared_ptr<hector_gamepad_manager_plugins::MoveitPlugin> plugin_;

  // Action servers for testing
  std::shared_ptr<hector_testing_utils::TestActionServer<moveit_msgs::action::MoveGroup>> moveit_server_;
  std::shared_ptr<hector_testing_utils::TestActionServer<control_msgs::action::FollowJointTrajectory>>
      trajectory_server_;

  // Captured goals for verification
  std::optional<moveit_msgs::action::MoveGroup::Goal> last_moveit_goal_;
  std::optional<control_msgs::action::FollowJointTrajectory::Goal> last_trajectory_goal_;
  std::atomic<int> moveit_goal_count_{ 0 };
  std::atomic<int> trajectory_goal_count_{ 0 };

  void SetUp() override
  {
    HectorTestFixture::SetUp();

    // Create action servers first (on tester_node_ which is already in executor)
    createActionServers();

    // Create plugin node with parameters set directly (avoid YAML file path issues)
    rclcpp::NodeOptions opts;
    opts.parameter_overrides( {
        rclcpp::Parameter( "moveit_plugin.start_controllers", std::vector<std::string>{} ),
        rclcpp::Parameter( "moveit_plugin.via_pose_ids", std::vector<std::string>{ "via_target" } ),
        rclcpp::Parameter( "moveit_plugin.via_target.group", std::string( "arm" ) ),
        rclcpp::Parameter( "moveit_plugin.via_target.target_pose", std::string( "target" ) ),
        rclcpp::Parameter( "moveit_plugin.via_target.via_pose", std::string( "via" ) ),
        rclcpp::Parameter( "moveit_plugin.controllers.arm",
                           std::string( "trajectory_controller/follow_joint_trajectory" ) ),
    } );
    plugin_node_ = std::make_shared<rclcpp::Node>( "moveit_plugin_test", opts );
    executor_->add_node( plugin_node_ );

    blackboard_ = std::make_shared<hector_gamepad_plugin_interface::Blackboard>();
    feedback_manager_ = std::make_shared<hector_gamepad_plugin_interface::FeedbackManager>();
    controller_orchestrator_ =
        std::make_shared<controller_orchestrator::ControllerOrchestrator>( plugin_node_ );

    plugin_ = std::make_shared<hector_gamepad_manager_plugins::MoveitPlugin>();
    plugin_->initializePlugin( plugin_node_, kPluginId, blackboard_, feedback_manager_,
                               controller_orchestrator_ );
    plugin_->activate();

    setConfig( kTargetId, kTargetPose );
    setConfig( kParkedId, kParkedPose );

    plugin_->robot_description_ = kUrdf;
    plugin_->robot_description_semantic_ = kSrdf;
    plugin_->initializeNamedPoses();

    // Verify trajectory client was created (via_poses loaded successfully)
    ASSERT_FALSE( plugin_->trajectory_clients_.empty() )
        << "Trajectory client was not created. Check that via_pose parameters were loaded.";

    // Wait for action client connections
    ASSERT_TRUE( moveit_server_->wait_for_client( *executor_, 5s ) )
        << "MoveIt action server did not receive client connection";
    ASSERT_TRUE( trajectory_server_->wait_for_client( *executor_, 5s ) )
        << "Trajectory action server did not receive client connection";
  }

  rclcpp::Node::SharedPtr plugin_node_;

  void createActionServers()
  {
    using MoveGroup = moveit_msgs::action::MoveGroup;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

    // MoveIt action server
    auto moveit_goal_callback = [this]( const rclcpp_action::GoalUUID &,
                                        const std::shared_ptr<const MoveGroup::Goal> goal ) {
      last_moveit_goal_ = *goal;
      moveit_goal_count_++;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto moveit_cancel_callback =
        []( const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>> ) {
          return rclcpp_action::CancelResponse::ACCEPT;
        };

    auto moveit_accepted_callback =
        []( const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>> goal_handle ) {
          std::thread( [goal_handle]() {
            auto result = std::make_shared<MoveGroup::Result>();
            result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
            goal_handle->succeed( result );
          } ).detach();
        };

    moveit_server_ = tester_node_->create_test_action_server<MoveGroup>(
        "/move_action", moveit_goal_callback, moveit_cancel_callback, moveit_accepted_callback );

    // Trajectory action server
    auto trajectory_goal_callback =
        [this]( const rclcpp_action::GoalUUID &,
                const std::shared_ptr<const FollowJointTrajectory::Goal> goal ) {
          last_trajectory_goal_ = *goal;
          trajectory_goal_count_++;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

    auto trajectory_cancel_callback =
        []( const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> ) {
          return rclcpp_action::CancelResponse::ACCEPT;
        };

    auto trajectory_accepted_callback =
        []( const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle ) {
          std::thread( [goal_handle]() {
            auto result = std::make_shared<FollowJointTrajectory::Result>();
            result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
            goal_handle->succeed( result );
          } ).detach();
        };

    trajectory_server_ = tester_node_->create_test_action_server<FollowJointTrajectory>(
        "/trajectory_controller/follow_joint_trajectory", trajectory_goal_callback,
        trajectory_cancel_callback, trajectory_accepted_callback );
  }

  void setConfig( const std::string &id, const std::string &pose )
  {
    const std::string prefix = std::string( kPluginId ) + "_" + id + "/";
    blackboard_->set( prefix + "group", std::string( kGroup ) );
    blackboard_->set( prefix + "pose", pose );
    blackboard_->set( prefix + "inverted_pose", pose );
  }

  void setJointStateForPose( const std::string &pose_name )
  {
    const auto key = plugin_->toGroupPoseName( kGroup, pose_name );
    auto it = plugin_->named_poses_.find( key );
    EXPECT_NE( it, plugin_->named_poses_.end() );
    if ( it == plugin_->named_poses_.end() ) {
      return;
    }

    sensor_msgs::msg::JointState state;
    for ( const auto &jc : it->second.joint_constraints ) {
      state.name.push_back( jc.joint_name );
      state.position.push_back( jc.position );
    }
    plugin_->joint_state_ = state;
  }

  void expectTrajectoryMatchesPose( const control_msgs::action::FollowJointTrajectory::Goal &goal,
                                    const std::string &pose_name )
  {
    const auto key = plugin_->toGroupPoseName( kGroup, pose_name );
    auto it = plugin_->named_poses_.find( key );
    ASSERT_NE( it, plugin_->named_poses_.end() );

    const auto expected = constraintsToMap( it->second.joint_constraints );
    ASSERT_FALSE( goal.trajectory.points.empty() );
    const auto actual = trajectoryToMap( goal.trajectory );

    EXPECT_EQ( expected.size(), actual.size() );
    for ( const auto &entry : expected ) {
      auto actual_it = actual.find( entry.first );
      ASSERT_NE( actual_it, actual.end() );
      EXPECT_NEAR( actual_it->second, entry.second, 1e-6 );
    }
  }

  void expectMoveItMatchesPose( const moveit_msgs::action::MoveGroup::Goal &goal,
                                const std::string &pose_name )
  {
    const auto key = plugin_->toGroupPoseName( kGroup, pose_name );
    auto it = plugin_->named_poses_.find( key );
    ASSERT_NE( it, plugin_->named_poses_.end() );

    ASSERT_FALSE( goal.request.goal_constraints.empty() );
    EXPECT_EQ( goal.request.group_name, kGroup );

    const auto expected = constraintsToMap( it->second.joint_constraints );
    const auto actual = constraintsToMap( goal.request.goal_constraints.front().joint_constraints );

    EXPECT_EQ( expected.size(), actual.size() );
    for ( const auto &entry : expected ) {
      auto actual_it = actual.find( entry.first );
      ASSERT_NE( actual_it, actual.end() );
      EXPECT_NEAR( actual_it->second, entry.second, 1e-6 );
    }
  }

  void setReadyState()
  {
    plugin_->state_ = hector_gamepad_manager_plugins::MoveitPlugin::State::CONTROLLER_SWITCH;
  }

  void resetGoalCounts()
  {
    moveit_goal_count_ = 0;
    trajectory_goal_count_ = 0;
    last_moveit_goal_.reset();
    last_trajectory_goal_.reset();
  }

  bool waitForTrajectoryGoal( std::chrono::milliseconds timeout = 1s )
  {
    return executor_->spin_until( [this]() { return trajectory_goal_count_ > 0; }, timeout );
  }

  bool waitForMoveItGoal( std::chrono::milliseconds timeout = 1s )
  {
    return executor_->spin_until( [this]() { return moveit_goal_count_ > 0; }, timeout );
  }
};

// =============================================================================
// Via Pose Trajectory Tests
// =============================================================================

TEST_F( MoveitPluginTest, TrajectoryFromTargetToViaPose )
{
  setJointStateForPose( kTargetPose );
  setReadyState();
  resetGoalCounts();

  plugin_->handleHold( "go_to_pose", kParkedId );

  ASSERT_TRUE( waitForTrajectoryGoal() ) << "Trajectory goal was not sent";
  ASSERT_TRUE( last_trajectory_goal_.has_value() );
  expectTrajectoryMatchesPose( *last_trajectory_goal_, kViaPose );

  EXPECT_EQ( moveit_goal_count_, 0 ) << "MoveIt should not be used for via pose transitions";
  EXPECT_EQ( plugin_->state_,
             hector_gamepad_manager_plugins::MoveitPlugin::State::TRAJECTORY_CONTROL );
}

TEST_F( MoveitPluginTest, TrajectoryFromViaToTargetPose )
{
  setJointStateForPose( kViaPose );
  setReadyState();
  resetGoalCounts();

  plugin_->handleHold( "go_to_pose", kTargetId );

  ASSERT_TRUE( waitForTrajectoryGoal() ) << "Trajectory goal was not sent";
  ASSERT_TRUE( last_trajectory_goal_.has_value() );
  expectTrajectoryMatchesPose( *last_trajectory_goal_, kTargetPose );

  EXPECT_EQ( moveit_goal_count_, 0 ) << "MoveIt should not be used for via pose transitions";
  EXPECT_EQ( plugin_->state_,
             hector_gamepad_manager_plugins::MoveitPlugin::State::TRAJECTORY_CONTROL );
}

TEST_F( MoveitPluginTest, MoveItUsedToReachViaFromElsewhere )
{
  setJointStateForPose( kHomePose );
  setReadyState();
  resetGoalCounts();

  plugin_->handleHold( "go_to_pose", kTargetId );

  ASSERT_TRUE( waitForMoveItGoal() ) << "MoveIt goal was not sent";
  ASSERT_TRUE( last_moveit_goal_.has_value() );
  expectMoveItMatchesPose( *last_moveit_goal_, kViaPose );

  EXPECT_EQ( trajectory_goal_count_, 0 )
      << "Trajectory controller should not be used from home pose";
  EXPECT_EQ( plugin_->state_, hector_gamepad_manager_plugins::MoveitPlugin::State::EXECUTING );
}

TEST_F( MoveitPluginTest, MoveItUsedForNormalPoseTransitions )
{
  setJointStateForPose( kHomePose );
  setReadyState();
  resetGoalCounts();

  plugin_->handleHold( "go_to_pose", kParkedId );

  ASSERT_TRUE( waitForMoveItGoal() ) << "MoveIt goal was not sent";
  ASSERT_TRUE( last_moveit_goal_.has_value() );
  expectMoveItMatchesPose( *last_moveit_goal_, kParkedPose );

  EXPECT_EQ( trajectory_goal_count_, 0 )
      << "Trajectory controller should not be used for normal transitions";
  EXPECT_EQ( plugin_->state_, hector_gamepad_manager_plugins::MoveitPlugin::State::EXECUTING );
}

// =============================================================================
// Joint Type Detection Tests
// =============================================================================

TEST_F( MoveitPluginTest, IsJointContinuousDetectsRevoluteJoint )
{
  // joint1 is defined as "revolute" in kUrdf
  EXPECT_FALSE( plugin_->isJointContinuous( "joint1" ) );
}

TEST_F( MoveitPluginTest, IsJointContinuousDetectsContinuousJoint )
{
  // joint2 is defined as "continuous" in kUrdf
  EXPECT_TRUE( plugin_->isJointContinuous( "joint2" ) );
}

TEST_F( MoveitPluginTest, IsJointContinuousReturnsFalseForUnknownJoint )
{
  EXPECT_FALSE( plugin_->isJointContinuous( "unknown_joint" ) );
}

// =============================================================================
// getClosestJointPosition Tests for Non-Continuous (Revolute) Joints
// =============================================================================

TEST_F( MoveitPluginTest, GetClosestJointPositionRevoluteReturnsTargetDirectly )
{
  // For revolute joints, the target position should be returned directly
  // without any wrap-around logic
  sensor_msgs::msg::JointState state;
  state.name = { "joint1", "joint2" };
  state.position = { 3.0, 0.0 };
  plugin_->joint_state_ = state;

  // Target is -3.0, current is 3.0. For revolute joint, should return -3.0 exactly
  double result = plugin_->getClosestJointPosition( "joint1", -3.0 );
  EXPECT_NEAR( result, -3.0, 1e-6 );
}

TEST_F( MoveitPluginTest, GetClosestJointPositionRevoluteDoesNotWrapAround )
{
  sensor_msgs::msg::JointState state;
  state.name = { "joint1", "joint2" };
  state.position = { 0.1, 0.0 };
  plugin_->joint_state_ = state;

  // Target is 3.0, current is 0.1. Should return 3.0, not wrap to negative
  double result = plugin_->getClosestJointPosition( "joint1", 3.0 );
  EXPECT_NEAR( result, 3.0, 1e-6 );
}

// =============================================================================
// getClosestJointPosition Tests for Continuous Joints
// =============================================================================

TEST_F( MoveitPluginTest, GetClosestJointPositionContinuousWrapsToClosest )
{
  sensor_msgs::msg::JointState state;
  state.name = { "joint1", "joint2" };
  state.position = { 0.0, 3.0 }; // joint2 (continuous) at 3.0 rad
  plugin_->joint_state_ = state;

  // Target is -3.0, current is 3.0. For continuous joint, should find closest
  // equivalent: 3.0 + remainder(-3.0 - 3.0, 2*pi) = 3.0 + 0.283... ≈ 3.283
  double result = plugin_->getClosestJointPosition( "joint2", -3.0 );
  // The closest equivalent to -3.0 from 3.0 is approximately 3.28 (within one rotation)
  EXPECT_NEAR( result, 3.0 + std::remainder( -3.0 - 3.0, 2 * M_PI ), 1e-6 );
}

TEST_F( MoveitPluginTest, GetClosestJointPositionContinuousNoWrapWhenClose )
{
  sensor_msgs::msg::JointState state;
  state.name = { "joint1", "joint2" };
  state.position = { 0.0, 0.5 }; // joint2 (continuous) at 0.5 rad
  plugin_->joint_state_ = state;

  // Target is 1.0, current is 0.5. Should return approximately 1.0
  double result = plugin_->getClosestJointPosition( "joint2", 1.0 );
  EXPECT_NEAR( result, 1.0, 1e-6 );
}

TEST_F( MoveitPluginTest, GetClosestJointPositionContinuousHandles2PiOffset )
{
  sensor_msgs::msg::JointState state;
  state.name = { "joint1", "joint2" };
  state.position = { 0.0, 0.1 }; // joint2 (continuous) near zero
  plugin_->joint_state_ = state;

  // Target is 2*PI + 0.1 (one full rotation ahead), should return close to current
  double result = plugin_->getClosestJointPosition( "joint2", 2 * M_PI + 0.1 );
  EXPECT_NEAR( result, 0.1, 1e-6 );
}

TEST_F( MoveitPluginTest, GetClosestJointPositionReturnsNanForMissingJoint )
{
  sensor_msgs::msg::JointState state;
  state.name = { "joint1" };
  state.position = { 0.0 };
  plugin_->joint_state_ = state;

  double result = plugin_->getClosestJointPosition( "joint2", 1.0 );
  EXPECT_TRUE( std::isnan( result ) );
}

TEST_F( MoveitPluginTest, GetClosestJointPositionReturnsNanForEmptyState )
{
  plugin_->joint_state_ = sensor_msgs::msg::JointState();

  double result = plugin_->getClosestJointPosition( "joint1", 1.0 );
  EXPECT_TRUE( std::isnan( result ) );
}

// =============================================================================
// isAtPose Tests
// =============================================================================

TEST_F( MoveitPluginTest, IsAtPoseReturnsTrueWhenAtPose )
{
  setJointStateForPose( kHomePose );
  EXPECT_TRUE( plugin_->isAtPose( plugin_->toGroupPoseName( kGroup, kHomePose ) ) );
}

TEST_F( MoveitPluginTest, IsAtPoseReturnsFalseWhenNotAtPose )
{
  setJointStateForPose( kHomePose );
  EXPECT_FALSE( plugin_->isAtPose( plugin_->toGroupPoseName( kGroup, kTargetPose ) ) );
}

TEST_F( MoveitPluginTest, IsAtPoseContinuousJointRecognizes2PiEquivalent )
{
  // Set joint state where joint2 (continuous) is at 2*PI offset from target
  const auto key = plugin_->toGroupPoseName( kGroup, kHomePose );
  auto it = plugin_->named_poses_.find( key );
  ASSERT_NE( it, plugin_->named_poses_.end() );

  sensor_msgs::msg::JointState state;
  for ( const auto &jc : it->second.joint_constraints ) {
    state.name.push_back( jc.joint_name );
    if ( jc.joint_name == "joint2" ) {
      // Add 2*PI to continuous joint - should still be recognized as "at pose"
      state.position.push_back( jc.position + 2 * M_PI );
    } else {
      state.position.push_back( jc.position );
    }
  }
  plugin_->joint_state_ = state;

  EXPECT_TRUE( plugin_->isAtPose( key ) );
}

TEST_F( MoveitPluginTest, IsAtPoseRevoluteJointDoesNotRecognize2PiEquivalent )
{
  // Set joint state where joint1 (revolute) is at 2*PI offset from target
  const auto key = plugin_->toGroupPoseName( kGroup, kHomePose );
  auto it = plugin_->named_poses_.find( key );
  ASSERT_NE( it, plugin_->named_poses_.end() );

  sensor_msgs::msg::JointState state;
  for ( const auto &jc : it->second.joint_constraints ) {
    state.name.push_back( jc.joint_name );
    if ( jc.joint_name == "joint1" ) {
      // Add 2*PI to revolute joint - should NOT be recognized as "at pose"
      // because revolute joints have limits and 2*PI offset is a different position
      state.position.push_back( jc.position + 2 * M_PI );
    } else {
      state.position.push_back( jc.position );
    }
  }
  plugin_->joint_state_ = state;

  EXPECT_FALSE( plugin_->isAtPose( key ) );
}

TEST_F( MoveitPluginTest, IsAtPoseReturnsFalseForUnknownPose )
{
  setJointStateForPose( kHomePose );
  EXPECT_FALSE( plugin_->isAtPose( plugin_->toGroupPoseName( kGroup, "nonexistent_pose" ) ) );
}

TEST_F( MoveitPluginTest, IsAtPoseReturnsFalseForEmptyJointState )
{
  plugin_->joint_state_ = sensor_msgs::msg::JointState();
  EXPECT_FALSE( plugin_->isAtPose( plugin_->toGroupPoseName( kGroup, kHomePose ) ) );
}

// =============================================================================
// Trajectory Goal Position Tests - Verifies correct positions are sent
// =============================================================================

TEST_F( MoveitPluginTest, TrajectoryGoalUsesCorrectPositionForContinuousJoint )
{
  // Set current position with joint2 (continuous) at 2*PI offset
  sensor_msgs::msg::JointState state;
  state.name = { "joint1", "joint2" };
  state.position = { 1.0, 2 * M_PI + 1.0 }; // joint2 at 2*PI + 1.0
  plugin_->joint_state_ = state;

  // Pretend we're at target pose (which has joint2 at 1.0)
  // The trajectory should command joint2 to stay near current (2*PI + 1.0),
  // not jump to 1.0
  const auto key = plugin_->toGroupPoseName( kGroup, kTargetPose );
  plugin_->named_poses_[key].joint_constraints[0].position = 1.0; // joint1
  plugin_->named_poses_[key].joint_constraints[1].position = 1.0; // joint2

  // Simulate being at via pose to trigger trajectory control to target
  const auto via_key = plugin_->toGroupPoseName( kGroup, kViaPose );
  plugin_->named_poses_[via_key].joint_constraints[0].position = 1.0;
  plugin_->named_poses_[via_key].joint_constraints[1].position = 2 * M_PI + 1.0;

  setReadyState();
  resetGoalCounts();

  plugin_->handleHold( "go_to_pose", kTargetId );

  ASSERT_TRUE( waitForTrajectoryGoal() ) << "Trajectory goal was not sent";
  ASSERT_TRUE( last_trajectory_goal_.has_value() );

  const auto actual = trajectoryToMap( last_trajectory_goal_->trajectory );

  // joint1 (revolute) should get exact target position
  EXPECT_NEAR( actual.at( "joint1" ), 1.0, 1e-6 );

  // joint2 (continuous) should get closest equivalent to 1.0 from current 2*PI+1.0
  // which is approximately 2*PI + 1.0 (staying close to current)
  double expected_joint2 = ( 2 * M_PI + 1.0 ) + std::remainder( 1.0 - ( 2 * M_PI + 1.0 ), 2 * M_PI );
  EXPECT_NEAR( actual.at( "joint2" ), expected_joint2, 1e-6 );
}

// =============================================================================
// Trajectory Safety Check Tests - max_trajectory_joint_distance_
// =============================================================================

TEST_F( MoveitPluginTest, TrajectoryRejectedWhenJointDistanceExceedsLimit )
{
  // Set current position far from via pose (more than PI/8 away)
  sensor_msgs::msg::JointState state;
  state.name = { "joint1", "joint2" };
  state.position = { 1.0, 1.0 }; // Current position at target
  plugin_->joint_state_ = state;

  // Via pose is at 0.5, 0.5 - which is 0.5 rad away (> PI/8 ≈ 0.393)
  // This should be rejected by the safety check
  plugin_->max_trajectory_joint_distance_ = M_PI / 8.0;

  setReadyState();
  resetGoalCounts();

  // Try to move from target to via using trajectory controller
  bool result = plugin_->moveUsingTrajectoryController( kGroup, kViaPose );

  // Give some time for any async operations
  executor_->spin_some();

  EXPECT_FALSE( result );
  EXPECT_EQ( trajectory_goal_count_, 0 )
      << "No trajectory should be sent when distance check fails";
  EXPECT_EQ( moveit_goal_count_, 0 ) << "No MoveIt goal should be sent either";
  EXPECT_EQ( plugin_->state_, hector_gamepad_manager_plugins::MoveitPlugin::State::ABORTED );
}

TEST_F( MoveitPluginTest, TrajectoryAcceptedWhenJointDistanceWithinLimit )
{
  // Set current position very close to via pose (less than PI/8 away)
  sensor_msgs::msg::JointState state;
  state.name = { "joint1", "joint2" };
  state.position = { 0.5 + 0.1, 0.5 + 0.1 }; // 0.1 rad away from via pose
  plugin_->joint_state_ = state;

  plugin_->max_trajectory_joint_distance_ = M_PI / 8.0; // ~0.393 rad

  setReadyState();
  resetGoalCounts();

  bool result = plugin_->moveUsingTrajectoryController( kGroup, kViaPose );

  ASSERT_TRUE( waitForTrajectoryGoal() ) << "Trajectory goal should be sent";
  EXPECT_TRUE( result );
}

TEST_F( MoveitPluginTest, TrajectoryDistanceCheckCanBeDisabled )
{
  // Set current position far from via pose
  sensor_msgs::msg::JointState state;
  state.name = { "joint1", "joint2" };
  state.position = { 1.0, 1.0 }; // 0.5 rad away from via pose
  plugin_->joint_state_ = state;

  // Disable the safety check by setting to 0
  plugin_->max_trajectory_joint_distance_ = 0.0;

  setReadyState();
  resetGoalCounts();

  bool result = plugin_->moveUsingTrajectoryController( kGroup, kViaPose );

  ASSERT_TRUE( waitForTrajectoryGoal() ) << "Trajectory goal should be sent when check is disabled";
  EXPECT_TRUE( result );
}

TEST_F( MoveitPluginTest, TrajectoryRejectedWhenSingleJointExceedsLimit )
{
  // Set current position where only one joint exceeds the limit
  sensor_msgs::msg::JointState state;
  state.name = { "joint1", "joint2" };
  state.position = { 0.5, 1.5 }; // joint1 at via, joint2 far away (1.0 rad difference)
  plugin_->joint_state_ = state;

  plugin_->max_trajectory_joint_distance_ = M_PI / 8.0;

  setReadyState();
  resetGoalCounts();

  bool result = plugin_->moveUsingTrajectoryController( kGroup, kViaPose );

  // Give some time for any async operations
  executor_->spin_some();

  EXPECT_FALSE( result );
  EXPECT_EQ( trajectory_goal_count_, 0 ) << "No trajectory should be sent";
  EXPECT_EQ( plugin_->state_, hector_gamepad_manager_plugins::MoveitPlugin::State::ABORTED );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
