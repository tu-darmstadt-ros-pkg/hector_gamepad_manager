//
// Created by aljoscha-schmidt on 3/10/25.
//

#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_MOVEIT_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_MOVEIT_PLUGIN_HPP

#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>
#include <hector_robot_pose_preview_msgs/srv/show_named_pose.hpp>
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace hector_gamepad_manager_plugins
{
class MoveitPlugin final : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{

  enum class State { IDLE, CONTROLLER_SWITCH, EXECUTING };

public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  void handlePress( const std::string &function, const std::string &id ) override;
  void handleRelease( const std::string &function, const std::string &id ) override;
  void handleHold( const std::string &function, const std::string &id ) override;

  void update() override;

  void activate() override;

  void deactivate() override;

private:
  void sendNamedPoseGoal( const std::string &move_group, const std::string &pose_name );
  void cancelGoal() const;
  void resultCallback(
      uint64_t request,
      const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult &result );
  void
  feedbackCallback( rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr,
                    const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> feedback );
  void goalResponseCallback(
      uint64_t request,
      const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr &goal_handle );
  void initializeNamedPoses();
  //! Shows/clears the transparent target pose preview. No-ops if the preview node is not running.
  void showPreview( const std::string &group, const std::string &pose ) const;
  void clearPreview() const;
  //! Tints the preview to signal that the target pose can not be reached. Cleared on release.
  void markPreviewInvalid() const;
  //! Whether MoveIt itself still has an active goal, e.g. one that is still planning after the
  //! relaying fold manager already reported its cancel. move_group crashes if a new goal arrives
  //! then (its move action shares one preempt flag between goals).
  bool isMoveitBusy();
  //! Controllers to activate for a group: `group_start_controllers.<group>`, else start_controllers.
  std::vector<std::string> startControllers( const std::string &group );
  //! Scaling factor for a group: `group_<name>.<group>` if set (0..1), else the global `default_value`.
  double groupScalingFactor( const std::string &name, const std::string &group, double default_value );

  static std::string toGroupPoseName( const std::string &group_name, const std::string &pose_name );
  std::pair<std::string, std::string> functionIdToGroupGroupAndPose( const std::string &function,
                                                                     const std::string &id ) const;
  bool active_ = false;
  State state_ = State::IDLE;
  double joint_tolerance_ = 0.05;
  double max_acceleration_scaling_factor_ = 0.3;
  double max_velocity_scaling_factor_ = 0.3;
  double planning_time_ = 1.0;
  // binding id of the pose button that started the current request, hold/release of other pose
  // buttons are ignored
  std::string active_id_;
  // sequence number of the last goal sent, and of the last one the user canceled (button release or
  // deactivation). Failures of user-canceled goals are expected and not signaled by vibration.
  uint64_t goal_request_ = 0;
  uint64_t canceled_request_ = 0;
  bool moveit_busy_ = false;
  bool initializedNamedPoses = false;
  std::string robot_description_;
  std::string robot_description_semantic_;
  std::map<std::string, moveit_msgs::msg::Constraints> named_poses_; // map <group>_<pose_name> to constraints
  std::vector<std::string> start_controllers_;
  sensor_msgs::msg::JointState joint_state_;
  std::string success_vibration_id_;
  std::string failure_vibration_id_;
  rclcpp::Node::SharedPtr node_;
  hector::ParameterSubscription velocity_scaling_factor_subscriber_;
  hector::ParameterSubscription acceleration_scaling_factor_subscriber_;
  hector::ParameterSubscription joint_tolerance_subscriber_;
  hector::ParameterSubscription planning_time_subscriber_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr moveit_status_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_semantic_subscriber_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr action_client_;
  rclcpp::Client<hector_robot_pose_preview_msgs::srv::ShowNamedPose>::SharedPtr preview_show_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr preview_clear_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr preview_set_valid_client_;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_MOVEIT_PLUGIN_HPP
