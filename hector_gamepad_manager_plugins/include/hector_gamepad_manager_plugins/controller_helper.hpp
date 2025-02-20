#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_CONTROLLER_HELPER_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_CONTROLLER_HELPER_HPP

#include <rclcpp/rclcpp.hpp>

namespace hector_gamepad_manager_plugins
{
    class ControllerHelper{

        public:

            ControllerHelper(const rclcpp::Node::SharedPtr &node);

            ~ControllerHelper();

            void switchControllers(std::vector<std::string> start_controllers, std::vector<std::string> stop_controllers);

        private:


            rclcpp::Client<controller_manager_msgs::srv::SwitchController> switch_controller_client_;

            rclcpp::Client<controller_manager_msgs::srv::ListControllers> list_controllers_client_;

            int max_switch_tries_;

            rclcpp::Node::SharedPtr& node_;
    }


}