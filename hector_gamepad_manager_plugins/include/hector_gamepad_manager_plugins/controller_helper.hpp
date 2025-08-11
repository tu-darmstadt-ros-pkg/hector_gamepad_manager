#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_CONTROLLER_HELPER_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_CONTROLLER_HELPER_HPP

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include <controller_orchestrator/controller_orchestrator.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hector_gamepad_manager_plugins
{
class ControllerHelper
{

public:
  ControllerHelper() = default;
  ~ControllerHelper() = default;

  void initialize( const rclcpp::Node::SharedPtr &node, std::string plugin_name );

  void switchControllers( std::vector<std::string> start_controllers ) const;

private:
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> controller_orchestrator_;
};
} // namespace hector_gamepad_manager_plugins

#endif