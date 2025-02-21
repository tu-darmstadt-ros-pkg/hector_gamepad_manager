#include "hector_gamepad_manager_plugins/controller_helper.hpp"

namespace hector_gamepad_manager_plugins
{

    void ControllerHelper::initialize(const rclcpp::Node::SharedPtr &node, std::string plugin_name){

        node_ = node;

        std::string switch_controllers_srv = "/" + node_->get_parameter("robot_namespace").as_string() + "/controller_manager/switch_controller";
        switch_controller_client_ = node->create_client<controller_manager_msgs::srv::SwitchController>(switch_controllers_srv);
        std::string list_controllers_srv = "/" + node_->get_parameter("robot_namespace").as_string() + "/controller_manager/list_controllers";

        list_controllers_client_ = node->create_client<controller_manager_msgs::srv::ListControllers>(list_controllers_srv);

        RCLCPP_INFO(node_->get_logger(), "Service name %s", switch_controllers_srv.c_str());
        max_switch_tries_ = 5;
        max_wait_on_srv_tries_ = 5;
        switch_sleep_rate_ = 100;

        plugin_name_ = plugin_name;
    }

    bool ControllerHelper::switchControllers(std::vector<std::string> start_controllers, std::vector<std::string> stop_controllers){

        if (start_controllers.empty() && stop_controllers.empty())
            return true;

        std::chrono::nanoseconds wait_dur = std::chrono::nanoseconds(1000000000);
        int wait_on_srv_tries = 0;
        while (!switch_controller_client_->wait_for_service(wait_dur) && !list_controllers_client_->wait_for_service(wait_dur)) {

            if (!rclcpp::ok()) 
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for control manager services. Exiting.");

            wait_on_srv_tries++;
            if(wait_on_srv_tries > max_wait_on_srv_tries_){
                RCLCPP_ERROR(node_->get_logger(), "Maximum waiting for service tries exceeded");
                return false;
            }
        }

        auto list_result = list_controllers_client_->async_send_request(std::make_shared<controller_manager_msgs::srv::ListControllers::Request> ());

        rclcpp::spin_until_future_complete(node_, list_result);

        RCLCPP_INFO(node_->get_logger(), "Future result received");

        auto list_response = list_result.get();
        // get controller list to check if the controllers are already running or stopped
        if (!list_response->controller.empty())
        {

            for (auto& controller : list_response->controller)
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
        return true;

    // fill service request

    auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_request->activate_controllers = start_controllers;
    switch_request->deactivate_controllers = stop_controllers;
    switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    switch_request->activate_asap = false;
    switch_request->timeout = rclcpp::Duration(0, 0);

    rclcpp::Rate sleep_rate(switch_sleep_rate_);

    // try to switch controllers several times
    for (int i = 0; i < max_switch_tries_; i++)
    {
        auto switch_result = switch_controller_client_->async_send_request(switch_request);

        rclcpp::spin_until_future_complete(node_, switch_result);

        // try to switch controller, if successful: break
        if (switch_result.get()->ok)
        {
            RCLCPP_INFO(node_->get_logger(), "Successful controller switch for plugin %s", plugin_name_.c_str());
            return true;
        }

        RCLCPP_INFO(node_->get_logger(), "Retrying controller switch for plugin %s", plugin_name_.c_str());
        sleep_rate.sleep();
    }

    RCLCPP_ERROR(node_->get_logger(), "Failed to switch controllers for plugin %s", plugin_name_.c_str());
    return false;

    }

}