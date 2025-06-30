#include "hector_gamepad_manager_plugins/controller_helper.hpp"
#include <controller_orchestrator/controller_orchestrator.hpp>

namespace hector_gamepad_manager_plugins
{

void ControllerHelper::initialize( const rclcpp::Node::SharedPtr &node, std::string plugin_name )
{

  node_ = node;
  controller_orchestrator_ = std::make_shared<controller_orchestrator::ControllerOrchestrator>(
      node_, "/" + node_->get_parameter( "robot_namespace" ).as_string() + "/controller_manager" );

  regular_srv_timeout_ = 10000;

  plugin_name_ = plugin_name;
}

void ControllerHelper::switchControllers( std::vector<std::string> start_controllers )
{
  if ( start_controllers.empty() )
    return;

  (void)controller_orchestrator_->smartSwitchController( start_controllers, regular_srv_timeout_ );
}

void ControllerHelper::controllerListCb(
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture response,
    std::vector<std::string> start_controllers )
{

  auto list_response = response.get();

  // Get controller list to check if the controllers are already running or stopped
  if ( !list_response->controller.empty() ) {

    for ( auto &controller : list_response->controller ) {

      // check if current controller name is in the start list
      auto start_iter =
          std::find( start_controllers.begin(), start_controllers.end(), controller.name );

      // if the controller was found in the start_controllers list and is already active, do not try to change the state later
      if ( start_iter != start_controllers.end() && controller.state == "active" ) {
        start_controllers.erase( start_iter );
      }
    }
  }

  // if everything is running/stopped, return
  if ( start_controllers.empty() )
    return;

  // fill service request
  auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  switch_request->activate_controllers = start_controllers;
  switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  switch_request->activate_asap = false;
  switch_request->timeout = rclcpp::Duration( 1, 0 );

  switch_controller_client_->async_send_request(
      switch_request,
      std::bind( &ControllerHelper::switchControllerCb, this, std::placeholders::_1 ) );
}

void ControllerHelper::switchControllerCb(
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture response )
{
  if ( !response.get()->ok ) {
    RCLCPP_ERROR( node_->get_logger(), "Controller switch for plugin %s failed",
                  plugin_name_.c_str() );
  }
}

} // namespace hector_gamepad_manager_plugins