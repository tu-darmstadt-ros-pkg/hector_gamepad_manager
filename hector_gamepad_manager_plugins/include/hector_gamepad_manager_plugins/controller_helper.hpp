#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_CONTROLLER_HELPER_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_CONTROLLER_HELPER_HPP

#include <rclcpp/rclcpp.hpp>
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"


namespace hector_gamepad_manager_plugins
{
    class ControllerHelper{

        public:

            ControllerHelper()=default;
            ~ControllerHelper()=default;

            void initialize(const rclcpp::Node::SharedPtr &node, std::string plugin_name);

            bool switchControllers(std::vector<std::string> start_controllers, std::vector<std::string> stop_controllers);

        private:

            rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
            rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;

            rclcpp::Node::SharedPtr node_;

            int max_switch_tries_;
            int max_wait_on_srv_tries_;

            int switch_retry_sleep_rate_;   // hz
            int regular_srv_timeout_;       // nanoseconds
            int init_srv_timeout_;          // nanoseconds

            std::string plugin_name_;  
     
    };
}

#endif 