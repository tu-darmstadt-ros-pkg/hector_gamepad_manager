//
// Created by aljoscha-schmidt on 3/8/25.
//

#include <hector_gamepad_manager_plugins/moveit_helper.hpp>
#include <srdfdom/model.h>
#include <std_msgs/msg/string.hpp>
#include <urdf_parser/urdf_parser.h>

namespace hector_gamepad_manager_plugins
{
void MoveitHelper::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  node_->declare_parameter<std::string>( "move_group_name", "arm_group" );
  move_group_name_ = node_->get_parameter( "move_group_name" ).as_string();

  // setup reconfigurable parameters
  velocity_scaling_factor_subscriber_ = hector::createReconfigurableParameter(
      node_, "max_velocity_scaling_factor", max_velocity_scaling_factor_,
      "Velocity scaling factor for moveit motion planning",
      hector::ReconfigurableParameterOptions<double>().onValidate(
          []( const auto &value ) { return value >= 0.0 && value <= 1.0; } ) );
  acceleration_scaling_factor_subscriber_ = hector::createReconfigurableParameter(
      node_, "max_acceleration_scaling_factor", max_acceleration_scaling_factor_,
      "Acceleration scaling factor for moveit motion planning",
      hector::ReconfigurableParameterOptions<double>().onValidate(
          []( const auto &value ) { return value >= 0.0 && value <= 1.0; } ) );
  joint_tolerance_subscriber_ = hector::createReconfigurableParameter(
      node_, "joint_tolerance", joint_tolerance_, "Joint tolerance for moveit motion planning",
      hector::ReconfigurableParameterOptions<double>().onValidate(
          []( const auto &value ) { return value >= 0.0; } ) );

  // setup action client
  action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
      node_, node_->get_effective_namespace() + "/move_action" );

  // setup robot description subscribers
  auto qos = rclcpp::QoS( 1 );
  qos.transient_local();
  robot_description_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
      "robot_description", qos, [this]( const std_msgs::msg::String::SharedPtr msg ) {
        robot_description_ = msg->data;
        robot_description_subscriber_.reset();
      } );
  robot_description_semantic_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
      "robot_description_semantic", qos, [this]( const std_msgs::msg::String::SharedPtr msg ) {
        robot_description_semantic_ = msg->data;
        robot_description_semantic_subscriber_.reset();
      } );
}

void MoveitHelper::sendNamedPoseGoal( const std::string &pose_name )
{
  if ( !initializedNamedPoses ) {
    initializeNamedPoses();
    initializedNamedPoses = true;
  }
  if ( named_poses_.find( pose_name ) == named_poses_.end() ) {
    RCLCPP_ERROR( node_->get_logger(), "Named pose %s not found.", pose_name.c_str() );
    return;
  }
  if ( !action_client_->wait_for_action_server( std::chrono::seconds( 5 ) ) ) {
    RCLCPP_ERROR( node_->get_logger(), "Action server not available after waiting" );
    return;
  }

  // update the joint tolerance
  for ( auto &joint_constraint : named_poses_[pose_name].joint_constraints ) {
    joint_constraint.tolerance_above = joint_tolerance_;
    joint_constraint.tolerance_below = joint_tolerance_;
  }

  // Create a goal message
  moveit_msgs::action::MoveGroup::Goal move_group_goal_;
  move_group_goal_.request.group_name = move_group_name_;
  move_group_goal_.request.goal_constraints.push_back( named_poses_[pose_name] );
  move_group_goal_.planning_options.plan_only = false;
  move_group_goal_.request.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
  move_group_goal_.request.max_velocity_scaling_factor = max_velocity_scaling_factor_;
  move_group_goal_.request.num_planning_attempts = 3;

  // Send the goal
  auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();
  send_goal_options.result_callback =
      std::bind( &MoveitHelper::resultCallback, this, std::placeholders::_1 );
  send_goal_options.feedback_callback = std::bind( &MoveitHelper::feedbackCallback, this,
                                                   std::placeholders::_1, std::placeholders::_2 );
  send_goal_options.goal_response_callback =
      std::bind( &MoveitHelper::goalResponseCallback, this, std::placeholders::_1 );
  this->action_client_->async_send_goal( move_group_goal_, send_goal_options );
}

void MoveitHelper::cancelGoal()
{
  if ( action_client_->wait_for_action_server( std::chrono::seconds( 5 ) ) ) {
    action_client_->async_cancel_all_goals();
  }
}

void MoveitHelper::resultCallback(
    const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult &result )
{
  if ( result.code != rclcpp_action::ResultCode::SUCCEEDED ) {
    RCLCPP_ERROR( node_->get_logger(), "Moveit action failed with result code %d",
                  static_cast<int>( result.code ) );
  }
}

void MoveitHelper::feedbackCallback(
    rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr,
    const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> feedback )
{
  RCLCPP_INFO( node_->get_logger(), "Moveit action feedback: %s", feedback->state.c_str() );
}

void MoveitHelper::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr &goal_handle )
{
  if ( !goal_handle ) {
    RCLCPP_ERROR( node_->get_logger(), "Goal was rejected by server" );
  } else {
    RCLCPP_INFO( node_->get_logger(), "Goal accepted by server, waiting for result" );
  }
}

void MoveitHelper::initializeNamedPoses()
{
  if ( robot_description_.empty() || robot_description_semantic_.empty() ) {
    RCLCPP_ERROR( node_->get_logger(), "Failed to get urdf and srdf file." );
    return;
  }
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF( robot_description_ );
  srdf::Model srdf_model;
  srdf_model.initString( *urdf_model, robot_description_semantic_ );
  const auto group_states = srdf_model.getGroupStates();
  for ( const auto &group_state : group_states ) {
    if ( group_state.group_ == move_group_name_ ) {
      moveit_msgs::msg::Constraints constraints;
      constraints.name = group_state.name_;
      for ( const auto &joint_state : group_state.joint_values_ ) {
        moveit_msgs::msg::JointConstraint joint_constraint;
        joint_constraint.joint_name = joint_state.first;
        joint_constraint.position = joint_state.second[0]; // assume single DOF joint
        joint_constraint.tolerance_above = joint_tolerance_;
        joint_constraint.tolerance_below = joint_tolerance_;
        joint_constraint.weight = 1.0;
        constraints.joint_constraints.push_back( joint_constraint );
      }
      named_poses_[group_state.name_] = constraints;
    }
  }
}

} // namespace hector_gamepad_manager_plugins