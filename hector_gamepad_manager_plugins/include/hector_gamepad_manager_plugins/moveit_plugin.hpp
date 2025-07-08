//
// Created by aljoscha-schmidt on 3/10/25.
//

#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_MOVEIT_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_MOVEIT_PLUGIN_HPP

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hector_gamepad_manager_plugins/controller_helper.hpp>
#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

namespace hector_gamepad_manager_plugins
{
class MoveitPlugin final : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  std::string getPluginName() override;

  void handlePress( const std::string &function ) override;
  void handleRelease( const std::string &function ) override;

  void update() override;

  void activate() override;

  void deactivate() override;

private:
  void sendNamedPoseGoal( const std::string &move_group, const std::string &pose_name );
  void cancelGoal() const;
  void resultCallback(
      const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult &result );
  void
  feedbackCallback( rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr,
                    const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> feedback );
  void goalResponseCallback(
      const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr &goal_handle );
  void initializeNamedPoses();
  double getJointPosition( const std::string &name ) const;
  double getNormalizedJointPosition( const std::string &name ) const;

  static std::string toGroupPoseName( const std::string &group_name, const std::string &pose_name );
  static std::pair<std::string, std::string> fromGroupPoseName( const std::string &group_pose_name );

  bool active_ = false;
  bool request_active_ = false;
  double joint_tolerance_ = 0.05;
  double max_acceleration_scaling_factor_ = 0.3;
  double max_velocity_scaling_factor_ = 0.3;
  bool initializedNamedPoses = false;
  std::string robot_description_;
  std::string robot_description_semantic_;
  std::map<std::string, moveit_msgs::msg::Constraints> named_poses_; // map <group>_<pose_name> to constraints
  std::vector<std::string> start_controllers_;
  ControllerHelper controller_helper_{};
  sensor_msgs::msg::JointState joint_state_;
  rclcpp::Node::SharedPtr node_;
  hector::ParameterSubscription velocity_scaling_factor_subscriber_;
  hector::ParameterSubscription acceleration_scaling_factor_subscriber_;
  hector::ParameterSubscription joint_tolerance_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_semantic_subscriber_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr action_client_;
};
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_MOVEIT_PLUGIN_HPP
