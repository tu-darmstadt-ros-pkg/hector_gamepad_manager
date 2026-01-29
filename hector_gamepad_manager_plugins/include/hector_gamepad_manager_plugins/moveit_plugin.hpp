//
// Created by aljoscha-schmidt on 3/10/25.
//

#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_MOVEIT_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_MOVEIT_PLUGIN_HPP

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

namespace hector_gamepad_manager_plugins
{
/**
 * @class MoveitPlugin
 * @brief Manages MoveIt! planning and direct trajectory execution via gamepad.
 * * Supports "Via Poses" to navigate through narrow passages or avoid self-collision
 * by combining MoveIt planning with direct joint-space execution via result callbacks.
 */
class MoveitPlugin final : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{
  /**
   * @enum State
   * @brief Internal state machine for managing asynchronous motion requests.
   */
  enum class State {
    IDLE,               ///< Plugin is inactive or waiting for press
    CONTROLLER_SWITCH,  ///< Ready to accept hold commands (controllers active)
    TRAJECTORY_CONTROL, ///< Direct FollowJointTrajectory action is in progress
    EXECUTING           ///< MoveIt MoveGroup action is in progress
  };

  struct ViaPose {
    std::string group;            ///< Planning group name
    std::string target_pose_name; ///< The final desired pose (e.g., "folded")
    std::string via_pose_name;    ///< The intermediate pose (e.g., "folded_partial")
  };

public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;
  void handlePress( const std::string &function, const std::string &id ) override;
  void handleRelease( const std::string &function, const std::string &id ) override;
  void handleHold( const std::string &function, const std::string &id ) override;
  void update() override;
  void activate() override;
  void deactivate() override;

private:
  /** @brief Sends a MoveGroup action goal to plan/execute to a pose. */
  void sendNamedPoseGoal( const std::string &move_group, const std::string &pose_name );

  /** @brief Sends a direct FollowJointTrajectory goal (bypassing MoveIt planning). */
  bool moveUsingTrajectoryController( const std::string &group, const std::string &pose_name );

  /** @brief Cancels all active action goals. */
  void cancelGoal() const;

  /** @brief Callback for MoveGroup completion. */
  void resultCallback(
      const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult &result );

  /** @brief Callback for Trajectory Controller completion. */
  void trajectoryResultCallback(
      const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult
          &result );

  void
  feedbackCallback( rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr,
                    const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> feedback );
  void goalResponseCallback(
      const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr &goal_handle );

  void initializeNamedPoses();
  void loadViaPoses();

  static std::string toGroupPoseName( const std::string &group_name, const std::string &pose_name );
  std::pair<std::string, std::string> functionIdToGroupGroupAndPose( const std::string &function,
                                                                     const std::string &id ) const;
  bool isAtPose( const std::string &group_pose_name );

  // Members
  bool active_ = false;
  State state_ = State::IDLE;
  double joint_tolerance_ = 0.05;
  double max_acceleration_scaling_factor_ = 0.3;
  double max_velocity_scaling_factor_ = 0.3;
  bool initializedNamedPoses = false;

  std::string robot_description_;
  std::string robot_description_semantic_;
  std::map<std::string, moveit_msgs::msg::Constraints> named_poses_;
  std::vector<ViaPose> via_poses_;
  std::vector<std::string> start_controllers_;

  sensor_msgs::msg::JointState joint_state_;
  rclcpp::Node::SharedPtr node_;

  hector::ParameterSubscription velocity_scaling_factor_subscriber_;
  hector::ParameterSubscription acceleration_scaling_factor_subscriber_;
  hector::ParameterSubscription joint_tolerance_subscriber_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_semantic_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr action_client_;
  std::map<std::string, rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr>
      trajectory_clients_;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_MOVEIT_PLUGIN_HPP