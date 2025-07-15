//
// Created by aljoscha-schmidt on 3/8/25.
//

#include <hector_gamepad_manager_plugins/moveit_plugin.hpp>
#include <srdfdom/model.h>
#include <std_msgs/msg/string.hpp>
#include <urdf_parser/urdf_parser.h>

namespace hector_gamepad_manager_plugins
{
void MoveitPlugin::initialize( const rclcpp::Node::SharedPtr &node_robot_ns )
{
  node_ = node_robot_ns;
  const std::string plugin_name = getPluginName();
  node_->declare_parameter<std::vector<std::string>>( plugin_name + ".start_controllers" );
  node_->declare_parameter<std::vector<std::string>>( plugin_name + ".stop_controllers" );
  start_controllers_ = node_->get_parameter( plugin_name + ".start_controllers" ).as_string_array();
  stop_controllers_ = node_->get_parameter( plugin_name + ".stop_controllers" ).as_string_array();
  // setup reconfigurable parameters
  velocity_scaling_factor_subscriber_ = hector::createReconfigurableParameter(
      node_, plugin_name + ".max_velocity_scaling_factor", std::ref( max_velocity_scaling_factor_ ),
      "Velocity scaling factor for moveit motion planning",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value >= 0.0 && value <= 1.0; } ) );
  acceleration_scaling_factor_subscriber_ = hector::createReconfigurableParameter(
      node_, plugin_name + ".max_acceleration_scaling_factor",
      std::ref( max_acceleration_scaling_factor_ ),
      "Acceleration scaling factor for moveit motion planning",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value >= 0.0 && value <= 1.0; } ) );
  joint_tolerance_subscriber_ = hector::createReconfigurableParameter(
      node_, plugin_name + ".joint_tolerance", std::ref( joint_tolerance_ ),
      "Joint tolerance for moveit motion planning",
      hector::ParameterOptions<double>().onValidate(
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
  controller_helper_.initialize( node_robot_ns, plugin_name );
  joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this]( const sensor_msgs::msg::JointState::SharedPtr msg ) { joint_state_ = *msg; } );
}

std::string MoveitPlugin::getPluginName() { return "moveit_plugin"; }

void MoveitPlugin::handlePress( const std::string &function )
{
  if ( !active_ )
    return;
  if ( !initializedNamedPoses ) {
    initializeNamedPoses();
    initializedNamedPoses = true;
  }
  if ( request_active_ ) {
    RCLCPP_WARN( node_->get_logger(), "Moveit action still active. Ignoring new request." );
    return;
  }
  // function is <group_name>_<pose_name>
  if ( named_poses_.count( function ) > 0 ) {
    controller_helper_.switchControllers( start_controllers_, stop_controllers_ );
    const auto [group, pose] = fromGroupPoseName( function );
    RCLCPP_WARN( node_->get_logger(), "Start Moveit Planning & Execution [%s]", function.c_str() );
    request_active_ = true;
    sendNamedPoseGoal( group, pose );
  } else {
    RCLCPP_WARN( node_->get_logger(), "No pose named %s found", function.c_str() );
  }
}

void MoveitPlugin::handleRelease( const std::string &function )
{
  if ( !active_ )
    return;
  // function is <group_name>_<pose_name>
  if ( named_poses_.count( function ) > 0 ) {
    cancelGoal(); // cancels all current goals
  }
}

void MoveitPlugin::update() { }

void MoveitPlugin::activate() { active_ = true; }

void MoveitPlugin::deactivate()
{
  active_ = false;
  cancelGoal(); // cancels all Goals
}

void MoveitPlugin::sendNamedPoseGoal( const std::string &move_group, const std::string &pose_name )
{
  std::string group_pose_name = toGroupPoseName( move_group, pose_name );
  if ( named_poses_.find( group_pose_name ) == named_poses_.end() ) {
    RCLCPP_ERROR( node_->get_logger(), "Named pose %s not found.", pose_name.c_str() );
    return;
  }
  if ( !action_client_->wait_for_action_server( std::chrono::seconds( 5 ) ) ) {
    RCLCPP_ERROR( node_->get_logger(), "Action server not available after waiting" );
    return;
  }

  // update the joint tolerance
  for ( auto &joint_constraint : named_poses_[group_pose_name].joint_constraints ) {
    joint_constraint.tolerance_above = joint_tolerance_;
    joint_constraint.tolerance_below = joint_tolerance_;
  }

  // Create a goal message
  moveit_msgs::action::MoveGroup::Goal move_group_goal_;
  move_group_goal_.request.group_name = move_group;
  move_group_goal_.request.goal_constraints.push_back( named_poses_[group_pose_name] );
  move_group_goal_.planning_options.plan_only = false;
  move_group_goal_.request.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
  move_group_goal_.request.max_velocity_scaling_factor = max_velocity_scaling_factor_;
  move_group_goal_.request.num_planning_attempts = 3;

  // make sure flippers don't flip the robot
  // side effect - flipper motions not possible if start state in [-3/4 pi, -pi]
  if ( move_group.find( "flipper" ) != std::string::npos ) {
    // add path constraints so that flipper joints must be within -3/4 pi and pi
    moveit_msgs::msg::JointConstraint joint_constraint;
    for ( const auto &joint : named_poses_[group_pose_name].joint_constraints ) {
      joint_constraint.joint_name = joint.joint_name;
      joint_constraint.position = 0;
      joint_constraint.tolerance_above = M_PI;
      joint_constraint.tolerance_below = 3.0 * M_PI / 4.0;
      joint_constraint.weight = 1.0;
      move_group_goal_.request.path_constraints.joint_constraints.push_back( joint_constraint );
    }
  }

  // Send the goal
  auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();
  send_goal_options.result_callback =
      std::bind( &MoveitPlugin::resultCallback, this, std::placeholders::_1 );
  send_goal_options.feedback_callback = std::bind( &MoveitPlugin::feedbackCallback, this,
                                                   std::placeholders::_1, std::placeholders::_2 );
  send_goal_options.goal_response_callback =
      std::bind( &MoveitPlugin::goalResponseCallback, this, std::placeholders::_1 );
  this->action_client_->async_send_goal( move_group_goal_, send_goal_options );
}

void MoveitPlugin::cancelGoal() const
{
  if ( action_client_->wait_for_action_server( std::chrono::seconds( 5 ) ) ) {
    action_client_->async_cancel_all_goals();
  }
}

void MoveitPlugin::resultCallback(
    const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult &result )
{
  if ( result.code != rclcpp_action::ResultCode::SUCCEEDED ) {
    RCLCPP_ERROR( node_->get_logger(), "Moveit action failed with result code %d",
                  static_cast<int>( result.code ) );
  }
  request_active_ = false;
}

void MoveitPlugin::feedbackCallback(
    rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr,
    const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> feedback )
{
  RCLCPP_INFO( node_->get_logger(), "Moveit action feedback: %s", feedback->state.c_str() );
}

void MoveitPlugin::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr &goal_handle )
{
  if ( !goal_handle ) {
    RCLCPP_ERROR( node_->get_logger(), "Goal was rejected by server" );
    request_active_ = false;
  } else {
    RCLCPP_INFO( node_->get_logger(), "Goal accepted by server, waiting for result" );
  }
}

void MoveitPlugin::initializeNamedPoses()
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
    named_poses_[toGroupPoseName( group_state.group_, group_state.name_ )] = constraints;
  }
}

double MoveitPlugin::getJointPosition( const std::string &name ) const
{
  // read joint position from joint state
  for ( size_t i = 0; i < joint_state_.name.size(); i++ ) {
    if ( joint_state_.name[i] == name ) {
      return joint_state_.position[i];
    }
  }
  RCLCPP_ERROR( node_->get_logger(), "Joint %s not found in joint state", name.c_str() );
  return 0.0;
}

double MoveitPlugin::getNormalizedJointPosition( const std::string &name ) const
{
  double position = getJointPosition( name );
  // make sure in interval [-pi, pi]
  while ( position > M_PI ) { position -= 2 * M_PI; }
  while ( position < -M_PI ) { position += 2 * M_PI; }
  return position;
}

std::string MoveitPlugin::toGroupPoseName( const std::string &group_name,
                                           const std::string &pose_name )
{
  return group_name + "/" + pose_name;
}

std::pair<std::string, std::string> MoveitPlugin::fromGroupPoseName( const std::string &group_pose_name )
{
  const auto pos = group_pose_name.find_last_of( '/' );
  if ( pos == std::string::npos ) {
    return { "", "" };
  }
  return { group_pose_name.substr( 0, pos ), group_pose_name.substr( pos + 1 ) };
}

} // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::MoveitPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )