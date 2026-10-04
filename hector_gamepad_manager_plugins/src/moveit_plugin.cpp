//
// Created by aljoscha-schmidt on 3/8/25.
//

#include <hector_gamepad_manager_plugins/blackboard_keys.hpp>
#include <hector_gamepad_manager_plugins/moveit_plugin.hpp>
#include <srdfdom/model.h>
#include <std_msgs/msg/string.hpp>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>

namespace hector_gamepad_manager_plugins
{
void MoveitPlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  const std::string plugin_name = getPluginName();
  node_->declare_parameter<std::vector<std::string>>( plugin_name + ".start_controllers" );
  start_controllers_ = node_->get_parameter( plugin_name + ".start_controllers" ).as_string_array();
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
  planning_time_subscriber_ = hector::createReconfigurableParameter(
      node_, plugin_name + ".planning_time", std::ref( planning_time_ ),
      "Allowed planning time in seconds for moveit motion planning",
      hector::ParameterOptions<double>().onValidate(
          []( const auto &value ) { return value > 0.0; } ) );

  // setup action client
  node_->declare_parameter<std::string>( plugin_name + ".action_topic", "move_action" );
  const auto action_topic = node_->get_parameter( plugin_name + ".action_topic" ).as_string();
  action_client_ =
      rclcpp_action::create_client<moveit_msgs::action::MoveGroup>( node_, action_topic );

  // Status of MoveIt's own move action (not the one of action_topic, which may be a relay).
  node_->declare_parameter<std::string>( plugin_name + ".moveit_status_topic",
                                         "move_action/_action/status" );
  rclcpp::SubscriptionOptions status_options;
  // A crashed move_group never reports its goals as finished.
  status_options.event_callbacks.liveliness_callback =
      [this]( rclcpp::QOSLivelinessChangedInfo &info ) {
        if ( info.alive_count == 0 )
          moveit_busy_ = false;
      };
  moveit_status_subscriber_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      node_->get_parameter( plugin_name + ".moveit_status_topic" ).as_string(),
      rclcpp::QoS( 1 ).reliable().transient_local(),
      [this]( const action_msgs::msg::GoalStatusArray::SharedPtr msg ) {
        moveit_busy_ =
            std::any_of( msg->status_list.begin(), msg->status_list.end(),
                         []( const action_msgs::msg::GoalStatus &status ) {
                           return status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                                  status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                                  status.status == action_msgs::msg::GoalStatus::STATUS_CANCELING;
                         } );
      },
      status_options );

  // optional target pose preview (hector_robot_pose_preview), its services are private to that node
  node_->declare_parameter<std::string>( plugin_name + ".preview_node", "robot_pose_preview" );
  const auto preview_node = node_->get_parameter( plugin_name + ".preview_node" ).as_string();
  preview_show_client_ = node_->create_client<hector_robot_pose_preview_msgs::srv::ShowNamedPose>(
      preview_node + "/show_named_pose" );
  preview_clear_client_ = node_->create_client<std_srvs::srv::Trigger>( preview_node + "/clear" );
  preview_set_valid_client_ =
      node_->create_client<std_srvs::srv::SetBool>( preview_node + "/set_valid" );

  // setup vibration feedback patterns
  const std::string success_pattern_ns = plugin_name + ".vibration_success";
  const std::string failure_pattern_ns = plugin_name + ".vibration_failure";
  success_vibration_id_ = success_pattern_ns;
  failure_vibration_id_ = failure_pattern_ns;
  if ( feedback_manager_ ) {
    hector_gamepad_plugin_interface::VibrationPatternDefaults success_defaults;
    success_defaults.on_durations_sec = { 0.3 };
    success_defaults.off_durations_sec = { 0.0 };
    success_defaults.intensity = 0.8;
    success_defaults.cycle = false;
    feedback_manager_->createVibrationPattern( success_vibration_id_, success_defaults );

    hector_gamepad_plugin_interface::VibrationPatternDefaults failure_defaults;
    failure_defaults.on_durations_sec = { 0.3, 0.3 };
    failure_defaults.off_durations_sec = { 0.2, 0.0 };
    failure_defaults.intensity = 0.8;
    failure_defaults.cycle = false;
    feedback_manager_->createVibrationPattern( failure_vibration_id_, failure_defaults );
  }

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

void MoveitPlugin::handlePress( const std::string &function, const std::string &id )
{
  if ( !active_ )
    return;
  if ( !initializedNamedPoses ) {
    initializeNamedPoses();
    initializedNamedPoses = true;
  }
  if ( state_ != State::IDLE ) {
    RCLCPP_WARN( node_->get_logger(),
                 "[MoveitPlugin] Moveit action still active. Ignoring new request." );
    return;
  }
  if ( function == "go_to_pose" ) {
    auto [group, pose] = functionIdToGroupGroupAndPose( function, id );
    if ( named_poses_.count( toGroupPoseName( group, pose ) ) > 0 ) {
      showPreview( group, pose );
      active_id_ = id;
      blackboard_->set( kMoveitActiveGroupKey, group );
      state_ = State::CONTROLLER_SWITCH;
      activateControllers(
          startControllers( group ), [this]( bool success, const std::string &message ) {
            if ( success ) {
              RCLCPP_DEBUG( node_->get_logger(), "[MoveitPlugin] Controller Switch successful!" );
            } else {
              // the controller switch failed, go back to IDLE (but if we were in EXECUTING, stay in
              // EXECUTING) e.g. controller were switched manually while waiting for controller switch
              if ( state_ == State::CONTROLLER_SWITCH )
                state_ = State::IDLE;
              RCLCPP_WARN( node_->get_logger(), "[MoveitPlugin] Controller Switch failed: %s",
                           message.c_str() );
            }
          } );
    } else {
      RCLCPP_WARN( node_->get_logger(), "[MoveitPlugin] No pose named %s found in group %s",
                   pose.c_str(), group.c_str() );
    }
  }
}

void MoveitPlugin::handleHold( const std::string &function, const std::string &id )
{
  // wait until controller switch is done
  if ( state_ != State::CONTROLLER_SWITCH || id != active_id_ ) {
    return;
  }
  if ( isMoveitBusy() ) {
    RCLCPP_INFO_THROTTLE( node_->get_logger(), *node_->get_clock(), 1000,
                          "[MoveitPlugin] Waiting for the previous MoveIt goal to finish." );
    return;
  }
  auto [group, pose] = functionIdToGroupGroupAndPose( function, id );
  if ( function == "go_to_pose" && areControllersActive( startControllers( group ) ) ) {
    if ( named_poses_.count( toGroupPoseName( group, pose ) ) > 0 ) {
      RCLCPP_WARN( node_->get_logger(),
                   "[MoveitPlugin] Start Moveit Planning & Execution of pose [%s] in group [%s]",
                   pose.c_str(), group.c_str() );
      state_ = State::EXECUTING;
      sendNamedPoseGoal( group, pose );
    } else {
      RCLCPP_WARN( node_->get_logger(), "[MoveitPlugin] No pose named %s found in group %s",
                   pose.c_str(), group.c_str() );
    }
  }
}

void MoveitPlugin::handleRelease( const std::string &function, const std::string &id )
{
  // only the button that started the motion may stop it
  if ( !active_ || id != active_id_ )
    return;
  auto [group, pose] = functionIdToGroupGroupAndPose( function, id );
  if ( named_poses_.count( toGroupPoseName( group, pose ) ) > 0 ) {
    clearPreview();
    cancelGoal(); // cancels all current goals
  }
  canceled_request_ = goal_request_;
  active_id_.clear();
  blackboard_->erase( kMoveitActiveGroupKey );
  state_ = State::IDLE;
}

void MoveitPlugin::update() { }

void MoveitPlugin::activate() { active_ = true; }

void MoveitPlugin::deactivate()
{
  active_ = false;
  clearPreview();
  cancelGoal(); // cancels all Goals
  canceled_request_ = goal_request_;
  active_id_.clear();
  blackboard_->erase( kMoveitActiveGroupKey );
  state_ = State::IDLE;
}

void MoveitPlugin::sendNamedPoseGoal( const std::string &move_group, const std::string &pose_name )
{
  std::string group_pose_name = toGroupPoseName( move_group, pose_name );
  if ( named_poses_.find( group_pose_name ) == named_poses_.end() ) {
    RCLCPP_ERROR( node_->get_logger(), "[MoveitPlugin] Named pose %s not found.", pose_name.c_str() );
    return;
  }
  if ( !action_client_->wait_for_action_server( std::chrono::seconds( 5 ) ) ) {
    RCLCPP_ERROR( node_->get_logger(), "[MoveitPlugin] Action server not available after waiting" );
    markPreviewInvalid();
    if ( feedback_manager_ ) {
      feedback_manager_->setPatternActive( failure_vibration_id_, true );
    }
    state_ = State::IDLE;
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
  move_group_goal_.request.max_acceleration_scaling_factor = groupScalingFactor(
      "max_acceleration_scaling_factor", move_group, max_acceleration_scaling_factor_ );
  move_group_goal_.request.max_velocity_scaling_factor =
      groupScalingFactor( "max_velocity_scaling_factor", move_group, max_velocity_scaling_factor_ );
  move_group_goal_.request.num_planning_attempts = 3;
  move_group_goal_.request.allowed_planning_time = planning_time_;

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
  const uint64_t request = ++goal_request_;
  send_goal_options.result_callback =
      std::bind( &MoveitPlugin::resultCallback, this, request, std::placeholders::_1 );
  send_goal_options.feedback_callback = std::bind( &MoveitPlugin::feedbackCallback, this,
                                                   std::placeholders::_1, std::placeholders::_2 );
  send_goal_options.goal_response_callback =
      std::bind( &MoveitPlugin::goalResponseCallback, this, request, std::placeholders::_1 );
  this->action_client_->async_send_goal( move_group_goal_, send_goal_options );
}

void MoveitPlugin::cancelGoal() const
{
  if ( action_client_->wait_for_action_server( std::chrono::seconds( 5 ) ) ) {
    action_client_->async_cancel_all_goals();
  }
}

void MoveitPlugin::resultCallback(
    uint64_t request,
    const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult &result )
{
  if ( request <= canceled_request_ ) {
    // canceled by the user releasing the button: no failure feedback. move_group may report this as
    // aborted, e.g. when canceled during planning. Release already cleared the preview and state.
    RCLCPP_INFO( node_->get_logger(),
                 "[MoveitPlugin] Moveit action canceled by user (result code %d)",
                 static_cast<int>( result.code ) );
    return;
  }
  if ( feedback_manager_ ) {
    if ( result.code == rclcpp_action::ResultCode::SUCCEEDED ) {
      feedback_manager_->setPatternActive( success_vibration_id_, true );
    } else {
      feedback_manager_->setPatternActive( failure_vibration_id_, true );
    }
  }
  if ( result.code != rclcpp_action::ResultCode::SUCCEEDED ) {
    RCLCPP_ERROR( node_->get_logger(), "[MoveitPlugin] Moveit action failed with result code %d",
                  static_cast<int>( result.code ) );
    // planning/execution failed: keep showing the target (tinted) until the button is released
    if ( result.code == rclcpp_action::ResultCode::ABORTED )
      markPreviewInvalid();
    else
      clearPreview(); // canceled, e.g. by releasing the button
  }
  state_ = State::IDLE;
}

void MoveitPlugin::feedbackCallback(
    rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr,
    const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> feedback )
{
  RCLCPP_INFO( node_->get_logger(), "[MoveitPlugin] Moveit action feedback: %s",
               feedback->state.c_str() );
}

void MoveitPlugin::goalResponseCallback(
    uint64_t request,
    const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr &goal_handle )
{
  if ( !goal_handle && request <= canceled_request_ ) {
    RCLCPP_INFO( node_->get_logger(),
                 "[MoveitPlugin] Goal was rejected by server after user cancel" );
  } else if ( !goal_handle ) {
    RCLCPP_ERROR( node_->get_logger(), "[MoveitPlugin] Goal was rejected by server" );
    markPreviewInvalid(); // keep showing the target (tinted) until the button is released
    if ( feedback_manager_ ) {
      feedback_manager_->setPatternActive( failure_vibration_id_, true );
    }
    state_ = State::IDLE;
  } else {
    RCLCPP_INFO( node_->get_logger(), "Goal accepted by server, waiting for result" );
  }
}

void MoveitPlugin::showPreview( const std::string &group, const std::string &pose ) const
{
  if ( !preview_show_client_ || !preview_show_client_->service_is_ready() )
    return;
  auto request = std::make_shared<hector_robot_pose_preview_msgs::srv::ShowNamedPose::Request>();
  request->group = group;
  request->pose = pose;
  preview_show_client_->async_send_request( request );
}

void MoveitPlugin::clearPreview() const
{
  if ( !preview_clear_client_ || !preview_clear_client_->service_is_ready() )
    return;
  preview_clear_client_->async_send_request( std::make_shared<std_srvs::srv::Trigger::Request>() );
}

std::vector<std::string> MoveitPlugin::startControllers( const std::string &group )
{
  // Declared on first use, the groups are only known from the button args.
  const std::string param = getPluginName() + ".group_start_controllers." + group;
  if ( !node_->has_parameter( param ) )
    node_->declare_parameter<std::vector<std::string>>( param, start_controllers_ );
  return node_->get_parameter( param ).as_string_array();
}

double MoveitPlugin::groupScalingFactor( const std::string &name, const std::string &group,
                                         double default_value )
{
  // Declared on first use like group_start_controllers; a negative value means "not set".
  const std::string param = getPluginName() + ".group_" + name + "." + group;
  if ( !node_->has_parameter( param ) )
    node_->declare_parameter<double>( param, -1.0 );
  const double value = node_->get_parameter( param ).as_double();
  return value >= 0.0 && value <= 1.0 ? value : default_value;
}

bool MoveitPlugin::isMoveitBusy()
{
  // Nobody reports the goals of a move_group that went away as finished.
  if ( moveit_status_subscriber_->get_publisher_count() == 0 )
    moveit_busy_ = false;
  return moveit_busy_;
}

void MoveitPlugin::markPreviewInvalid() const
{
  if ( !preview_set_valid_client_ || !preview_set_valid_client_->service_is_ready() )
    return;
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;
  preview_set_valid_client_->async_send_request( request );
}

void MoveitPlugin::initializeNamedPoses()
{
  if ( robot_description_.empty() || robot_description_semantic_.empty() ) {
    RCLCPP_ERROR( node_->get_logger(), "[MoveitPlugin] Failed to get urdf and srdf file." );
    return;
  }
  const urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF( robot_description_ );
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

std::string MoveitPlugin::toGroupPoseName( const std::string &group_name,
                                           const std::string &pose_name )
{
  return group_name + "/" + pose_name;
}

std::pair<std::string, std::string>
MoveitPlugin::functionIdToGroupGroupAndPose( const std::string &function, const std::string &id ) const
{
  const auto &group = getConfigValueOr<std::string>( id, "group", "" );
  const auto &normal_pose = getConfigValueOr<std::string>( id, "pose", "" );
  const auto &inverted_pose = getConfigValueOr<std::string>( id, "inverted_pose", "" );
  const auto &inverted_steering = blackboard_->value_or<bool>( "inverted_steering", false );
  const auto &pose = inverted_steering ? inverted_pose : normal_pose;
  return { group, pose };
}

} // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::MoveitPlugin,
                        hector_gamepad_plugin_interface::GamepadFunctionPlugin )
