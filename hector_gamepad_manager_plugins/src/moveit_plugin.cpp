#include <hector_gamepad_manager_plugins/moveit_plugin.hpp>
#include <srdfdom/model.h>
#include <std_msgs/msg/string.hpp>
#include <urdf_parser/urdf_parser.h>

namespace hector_gamepad_manager_plugins
{

void MoveitPlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  const std::string plugin_name = getPluginName();

  node_->declare_parameter<std::vector<std::string>>( plugin_name + ".start_controllers", {} );
  start_controllers_ = node_->get_parameter( plugin_name + ".start_controllers" ).as_string_array();

  // Reconfigurable params
  velocity_scaling_factor_subscriber_ = hector::createReconfigurableParameter(
      node_, plugin_name + ".max_velocity_scaling_factor", std::ref( max_velocity_scaling_factor_ ),
      "Velocity scaling factor for moveit",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &v ) { return v >= 0.0 && v <= 1.0; } ) );

  acceleration_scaling_factor_subscriber_ = hector::createReconfigurableParameter(
      node_, plugin_name + ".max_acceleration_scaling_factor",
      std::ref( max_acceleration_scaling_factor_ ), "Acceleration scaling factor for moveit",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &v ) { return v >= 0.0 && v <= 1.0; } ) );

  joint_tolerance_subscriber_ = hector::createReconfigurableParameter(
      node_, plugin_name + ".joint_tolerance", std::ref( joint_tolerance_ ),
      "Joint tolerance for matching poses",
      hector::ParameterOptions<double>().onValidate( []( const auto &v ) { return v >= 0.0; } ) );

  loadViaPoses();

  action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
      node_, node_->get_effective_namespace() + "/move_action" );

  joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this]( const sensor_msgs::msg::JointState::SharedPtr msg ) { joint_state_ = *msg; } );

  auto qos = rclcpp::QoS( 1 ).transient_local();
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

void MoveitPlugin::loadViaPoses()
{
  const std::string plugin_name = getPluginName();
  node_->declare_parameter<std::vector<std::string>>( plugin_name + ".via_pose_ids", {} );
  auto via_ids = node_->get_parameter( plugin_name + ".via_pose_ids" ).as_string_array();

  for ( const auto &id : via_ids ) {
    ViaPose vp;
    vp.group = node_->get_parameter( plugin_name + "." + id + ".group" ).as_string();
    vp.target_pose_name = node_->get_parameter( plugin_name + "." + id + ".target_pose" ).as_string();
    vp.via_pose_name = node_->get_parameter( plugin_name + "." + id + ".via_pose" ).as_string();
    via_poses_.push_back( vp );

    std::string action_topic =
        node_->declare_parameter<std::string>( plugin_name + ".controllers." + vp.group, "" );
    if ( !action_topic.empty() && trajectory_clients_.count( vp.group ) == 0 ) {
      trajectory_clients_[vp.group] =
          rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>( node_,
                                                                                     action_topic );
    }
  }
}

void MoveitPlugin::handleHold( const std::string &function, const std::string &id )
{
  // Only proceed if controllers are active and we are ready for a new command
  if ( state_ != State::CONTROLLER_SWITCH )
    return;
  if ( function != "go_to_pose" || !areControllersActive( start_controllers_ ) )
    return;

  auto [group, goal_pose] = functionIdToGroupGroupAndPose( function, id );

  if ( isAtPose( goal_pose ) ) {
    RCLCPP_INFO( node_->get_logger(), "[MoveitPlugin] Already at goal pose [%s] for group [%s].",
                 goal_pose.c_str(), group.c_str() );
    state_ = State::IDLE;
    return;
  }

  for ( const auto &vp : via_poses_ ) {
    if ( vp.group == group && vp.target_pose_name == goal_pose ) {
      // Step A: We are at the intermediate via pose -> Move to final goal via Trajectory Controller
      if ( isAtPose( toGroupPoseName( group, vp.via_pose_name ) ) ) {
        RCLCPP_INFO( node_->get_logger(), "[MoveitPlugin] At via pose [%s]. Direct motion to [%s].",
                     vp.via_pose_name.c_str(), goal_pose.c_str() );
        state_ = State::TRAJECTORY_CONTROL;
        if ( !moveUsingTrajectoryController( group, goal_pose ) )
          state_ = State::CONTROLLER_SWITCH;
        return;
      }

      // Step B: We are elsewhere -> Plan to via pose via MoveIt
      RCLCPP_INFO( node_->get_logger(), "[MoveitPlugin] Planning to via pose [%s] for group [%s].",
                   vp.via_pose_name.c_str(), group.c_str() );
      state_ = State::EXECUTING;
      sendNamedPoseGoal( group, vp.via_pose_name );
      return;
    }
  }

  // Standard case: Plan directly to goal via MoveIt
  state_ = State::EXECUTING;
  sendNamedPoseGoal( group, goal_pose );
}

bool MoveitPlugin::moveUsingTrajectoryController( const std::string &group,
                                                  const std::string &pose_name )
{
  if ( trajectory_clients_.count( group ) == 0 )
    return false;

  auto goal = control_msgs::action::FollowJointTrajectory::Goal();
  const auto &constraints = named_poses_[toGroupPoseName( group, pose_name )].joint_constraints;

  trajectory_msgs::msg::JointTrajectoryPoint p;
  for ( const auto &jc : constraints ) {
    goal.trajectory.joint_names.push_back( jc.joint_name );
    p.positions.push_back( jc.position );
  }
  p.time_from_start = rclcpp::Duration::from_seconds( 2.0 );
  goal.trajectory.points.push_back( p );

  auto options =
      rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
  options.result_callback =
      std::bind( &MoveitPlugin::trajectoryResultCallback, this, std::placeholders::_1 );

  trajectory_clients_[group]->async_send_goal( goal, options );
  return true;
}

void MoveitPlugin::resultCallback(
    const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult &result )
{
  RCLCPP_INFO( node_->get_logger(),
               "[MoveitPlugin] MoveIt action finished. Returning to ready state." );
  state_ = State::CONTROLLER_SWITCH;
}

void MoveitPlugin::trajectoryResultCallback(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult &result )
{
  RCLCPP_INFO( node_->get_logger(),
               "[MoveitPlugin] Trajectory action finished. Returning to ready state." );
  state_ = State::CONTROLLER_SWITCH;
}

void MoveitPlugin::handlePress( const std::string &function, const std::string &id )
{
  if ( !active_ )
    return;
  if ( !initializedNamedPoses ) {
    initializeNamedPoses();
    initializedNamedPoses = true;
  }
  if ( state_ != State::IDLE )
    return;

  if ( function == "go_to_pose" ) {
    auto [group, pose] = functionIdToGroupGroupAndPose( function, id );
    if ( named_poses_.count( toGroupPoseName( group, pose ) ) > 0 ) {
      state_ = State::CONTROLLER_SWITCH;
      activateControllers( start_controllers_, [this]( bool success, const std::string &message ) {
        if ( !success && state_ == State::CONTROLLER_SWITCH )
          state_ = State::IDLE;
      } );
    }
  }
}

void MoveitPlugin::handleRelease( const std::string &, const std::string & )
{
  cancelGoal();
  state_ = State::IDLE;
}

bool MoveitPlugin::isAtPose( const std::string &group_pose_name )
{
  if ( named_poses_.count( group_pose_name ) == 0 || joint_state_.name.empty() )
    return false;
  const auto &constraints = named_poses_[group_pose_name].joint_constraints;

  for ( const auto &jc : constraints ) {
    auto it = std::find( joint_state_.name.begin(), joint_state_.name.end(), jc.joint_name );
    if ( it == joint_state_.name.end() )
      continue;
    size_t idx = std::distance( joint_state_.name.begin(), it );
    if ( std::abs( joint_state_.position[idx] - jc.position ) > joint_tolerance_ )
      return false;
  }
  return true;
}

void MoveitPlugin::sendNamedPoseGoal( const std::string &move_group, const std::string &pose_name )
{
  std::string group_pose_name = toGroupPoseName( move_group, pose_name );
  if ( named_poses_.find( group_pose_name ) == named_poses_.end() )
    return;

  moveit_msgs::action::MoveGroup::Goal goal;
  goal.request.group_name = move_group;

  auto constraints = named_poses_[group_pose_name];
  for ( auto &jc : constraints.joint_constraints ) {
    jc.tolerance_above = joint_tolerance_;
    jc.tolerance_below = joint_tolerance_;
  }

  goal.request.goal_constraints.push_back( constraints );
  goal.request.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
  goal.request.max_velocity_scaling_factor = max_velocity_scaling_factor_;

  auto options = rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();
  options.result_callback = std::bind( &MoveitPlugin::resultCallback, this, std::placeholders::_1 );
  options.goal_response_callback =
      std::bind( &MoveitPlugin::goalResponseCallback, this, std::placeholders::_1 );

  this->action_client_->async_send_goal( goal, options );
}

void MoveitPlugin::cancelGoal() const
{
  action_client_->async_cancel_all_goals();
  for ( auto &it : trajectory_clients_ ) it.second->async_cancel_all_goals();
}

void MoveitPlugin::initializeNamedPoses()
{
  if ( robot_description_.empty() || robot_description_semantic_.empty() )
    return;
  const urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF( robot_description_ );
  srdf::Model srdf_model;
  srdf_model.initString( *urdf_model, robot_description_semantic_ );

  for ( const auto &gs : srdf_model.getGroupStates() ) {
    moveit_msgs::msg::Constraints constraints;
    for ( const auto &jv : gs.joint_values_ ) {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = jv.first;
      jc.position = jv.second[0];
      jc.weight = 1.0;
      constraints.joint_constraints.push_back( jc );
    }
    named_poses_[toGroupPoseName( gs.group_, gs.name_ )] = constraints;
  }
}

void MoveitPlugin::update() { }
void MoveitPlugin::activate() { active_ = true; }
void MoveitPlugin::deactivate()
{
  active_ = false;
  cancelGoal();
  state_ = State::IDLE;
}
std::string MoveitPlugin::toGroupPoseName( const std::string &g, const std::string &p )
{
  return g + "/" + p;
}
void MoveitPlugin::feedbackCallback(
    rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr,
    const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> )
{
}
void MoveitPlugin::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr &goal_handle )
{
  if ( !goal_handle )
    state_ = State::CONTROLLER_SWITCH;
}
std::pair<std::string, std::string>
MoveitPlugin::functionIdToGroupGroupAndPose( const std::string &function, const std::string &id ) const
{
  const auto &group = getConfigValueOr<std::string>( id, "group", "" );
  const auto &normal_pose = getConfigValueOr<std::string>( id, "pose", "" );
  const auto &inverted_pose = getConfigValueOr<std::string>( id, "inverted_pose", "" );
  const auto &pose =
      blackboard_->value_or<bool>( "inverted_steering", false ) ? inverted_pose : normal_pose;
  return { group, pose };
}

} // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::MoveitPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )