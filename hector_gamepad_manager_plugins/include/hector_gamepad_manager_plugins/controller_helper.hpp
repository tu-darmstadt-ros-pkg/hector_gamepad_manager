#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_CONTROLLER_HELPER_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_CONTROLLER_HELPER_HPP

#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include <controller_orchestrator/controller_orchestrator.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hector_gamepad_manager_plugins
{
class ControllerHelper
{

public:
  ControllerHelper() = default;
  ~ControllerHelper() = default;

  void initialize( const rclcpp::Node::SharedPtr &node, const std::string &plugin_name );

  void switchControllers( const std::vector<std::string> &start_controllers ) const;

private:
  rclcpp::Subscription<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr ctrl_subscription_;

  std::vector<std::string> active_controllers_;
  rclcpp::Node::SharedPtr node_;
  std::string plugin_name_;
  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> controller_orchestrator_;
};
} // namespace hector_gamepad_manager_plugins

#endif