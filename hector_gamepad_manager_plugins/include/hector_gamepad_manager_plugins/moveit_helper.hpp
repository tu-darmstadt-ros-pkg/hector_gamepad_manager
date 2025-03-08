#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_MOVEIT_HELPER_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_MOVEIT_HELPER_HPP

#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>

namespace hector_gamepad_manager_plugins
{
class MoveitHelper
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node );
  void sendNamedPoseGoal( const std::string &pose_name );
  void cancelGoal();

private:
  void resultCallback(
      const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult &result );
  void
  feedbackCallback( rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr,
                    const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> feedback );
  void goalResponseCallback(
      const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr &goal_handle );
  void initializeNamedPoses();

  std::string move_group_name_;
  double joint_tolerance_ = 0.1;
  double max_acceleration_scaling_factor_ = 0.3;
  double max_velocity_scaling_factor_ = 0.3;
  bool initializedNamedPoses = false;
  std::string robot_description_;
  std::string robot_description_semantic_;
  std::map<std::string, moveit_msgs::msg::Constraints> named_poses_;
  rclcpp::Node::SharedPtr node_;
  hector::ReconfigurableParameterSubscription velocity_scaling_factor_subscriber_;
  hector::ReconfigurableParameterSubscription acceleration_scaling_factor_subscriber_;
  hector::ReconfigurableParameterSubscription joint_tolerance_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_semantic_subscriber_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr action_client_;
};
} // namespace hector_gamepad_manager_plugins
#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_MOVEIT_HELPER_HPP