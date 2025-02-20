#include "hector_gamepad_manager_plugins/controller_helper.hpp"

namespace hector_gamepad_manager_plugins
{

    ControllerHelper::ControllerHelper(const rclcpp::Node::SharedPtr &node){

        node_ = node;
        std::string switch_controllers_srv = node->get_namespace() + "/controller_manager/switch_controllers";
        switch_controller_client_ = node->create_client<controller_manager_msgs::srv::SwitchController>(switch_controllers_srv);
        std::string list_controllers_srv = node->get_namespace() + "/controller_manager/list_controllers";
        list_controllers_client_ = node->create_client<controller_manager_msgs::srv::ListControllers>(list_controllers_srv);
        max_switch_tries_ = 5;
    }

    void ControllerHelper::switchControllers(std::vector<std::string> start_controllers, std::vector<std::string> stop_controller){

        if (start_controllers.empty() && stop_controllers.empty())
            return;


        auto list_result = list_controllers_client_->async_send_request(std::make_shared<controller_manager_msgs::srv::ListControllers::Request> ());

        rclcpp::spin_until_future_complete(node_, list_result);
        // get controller list to check if the controllers are already running or stopped
        if (list_controllers_client_.call(list_controllers_srv_) && !list_controllers_srv_.response.controller.empty())
        {

            for (auto& controller : list_controllers_srv_.response.controller)
            {

                // check if current controller name is in one of the start/stop lists
                auto start_iter = std::find(start_controllers.begin(), start_controllers.end(), controller.name);
                auto stop_iter = std::find(stop_controllers.begin(), stop_controllers.end(), controller.name);

                // if the controller was found in one of the lists and has the desired state, do not try to change the state later
                if (start_iter != start_controllers.end() && controller.state == "running")
                {
                    start_controllers.erase(start_iter);
                } else if (stop_iter != stop_controllers.end()
                    && (controller.state == "stopped" || controller.state == "initialized"))
                {
                    stop_controllers.erase(stop_iter);
                }
            }
        }

    // if everything is running/stopped, return
    if (start_controllers.empty() && stop_controllers.empty())
    {
        return "";
    }

    // fill service request
    switch_controller_srv_.request.start_controllers = start_controllers;
    switch_controller_srv_.request.stop_controllers = stop_controllers;
    switch_controller_srv_.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
    switch_controller_srv_.request.start_asap = false;
    switch_controller_srv_.request.timeout = 0.0;

    }

}