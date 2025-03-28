#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_CONTROLLER_HELPER_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_CONTROLLER_HELPER_HPP

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include <rclcpp/rclcpp.hpp>

namespace hector_gamepad_manager_plugins
{
class ControllerHelper
{

public:
  ControllerHelper() = default;
  ~ControllerHelper() = default;

  void initialize( const rclcpp::Node::SharedPtr &node, std::string plugin_name );

  void switchControllers( std::vector<std::string> start_controllers,
                          std::vector<std::string> stop_controllers );

private:
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;

  void controllerListCb(
      rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture response,
      std::vector<std::string> start_controllers, std::vector<std::string> stop_controllers );
  void switchControllerCb(
      rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture response );

  rclcpp::Node::SharedPtr node_;

  int regular_srv_timeout_; // nanoseconds

  std::string plugin_name_;
};
} // namespace hector_gamepad_manager_plugins

#endif