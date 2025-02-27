#include "hector_gamepad_manager_plugins/controller_helper.hpp"

namespace hector_gamepad_manager_plugins
{

void ControllerHelper::initialize( const rclcpp::Node::SharedPtr &node, std::string plugin_name )
{

  node_ = node;

  std::string switch_controllers_srv = "/" + node_->get_parameter( "robot_namespace" ).as_string() +
                                       "/controller_manager/switch_controller";
  switch_controller_client_ =
      node->create_client<controller_manager_msgs::srv::SwitchController>( switch_controllers_srv );
  std::string list_controllers_srv = "/" + node_->get_parameter( "robot_namespace" ).as_string() +
                                     "/controller_manager/list_controllers";
  list_controllers_client_ =
      node->create_client<controller_manager_msgs::srv::ListControllers>( list_controllers_srv );

  max_switch_tries_ = 2;

  max_wait_on_srv_tries_ = 20;
  regular_srv_timeout_ = 10000; // 1s

  plugin_name_ = plugin_name;
}

void ControllerHelper::switchControllers( std::vector<std::string> start_controllers,
                                          std::vector<std::string> stop_controllers )
{

  if ( start_controllers.empty() && stop_controllers.empty() )
    return;

  std::chrono::nanoseconds wait_dur = std::chrono::nanoseconds( regular_srv_timeout_ );
  switch_controller_client_->wait_for_service( wait_dur );
  list_controllers_client_->wait_for_service( wait_dur );

  list_controllers_client_->async_send_request(
      std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(),
      [this, start_controllers, stop_controllers](
          rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture response ) {
        this->controller_list_cb( response, start_controllers, stop_controllers );
      } );
}

void ControllerHelper::controller_list_cb(
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture response,
    std::vector<std::string> start_controllers, std::vector<std::string> stop_controllers )
{

  auto list_response = response.get();

  // Get controller list to check if the controllers are already running or stopped
  if ( !list_response->controller.empty() ) {

    for ( auto &controller : list_response->controller ) {

      // check if current controller name is in one of the start/stop lists
      auto start_iter =
          std::find( start_controllers.begin(), start_controllers.end(), controller.name );
      auto stop_iter = std::find( stop_controllers.begin(), stop_controllers.end(), controller.name );

      // if the controller was found in one of the lists and has the desired state, do not try to change the state later
      if ( start_iter != start_controllers.end() && controller.state == "running" ) {
        start_controllers.erase( start_iter );
      } else if ( stop_iter != stop_controllers.end() &&
                  ( controller.state == "stopped" || controller.state == "initialized" ) ) {
        stop_controllers.erase( stop_iter );
      }
    }
  }

  // if everything is running/stopped, return
  if ( start_controllers.empty() && stop_controllers.empty() )
    return;

  // fill service request
  auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  switch_request->activate_controllers = start_controllers;
  switch_request->deactivate_controllers = stop_controllers;
  switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  switch_request->activate_asap = false;
  switch_request->timeout = rclcpp::Duration( 0, 0 );

  switch_controller_client_->async_send_request(
      switch_request,
      std::bind( &ControllerHelper::switch_controller_cb, this, std::placeholders::_1 ) );
}

void ControllerHelper::switch_controller_cb(
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture response )
{

  if ( response.get()->ok ) {
    RCLCPP_INFO( node_->get_logger(), "Successful controller switch for plugin %s",
                 plugin_name_.c_str() );
  }
}

} // namespace hector_gamepad_manager_plugins