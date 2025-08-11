#include "hector_gamepad_manager_plugins/controller_helper.hpp"
#include <controller_orchestrator/controller_orchestrator.hpp>

namespace hector_gamepad_manager_plugins
{

void ControllerHelper::initialize( const rclcpp::Node::SharedPtr &node,
                                   const std::string &plugin_name )
{

  node_ = node;
  plugin_name_ = plugin_name;
  controller_orchestrator_ = std::make_shared<controller_orchestrator::ControllerOrchestrator>(
      node_, "/" + node_->get_parameter( "robot_namespace" ).as_string() + "/controller_manager" );
}

void ControllerHelper::switchControllers( const std::vector<std::string> &start_controllers ) const
{
  if ( start_controllers.empty() )
    return;
  controller_orchestrator_->smartSwitchControllerAsync(
      start_controllers, [this]( bool success, const std::string &message ) {
        if ( success ) {
          RCLCPP_DEBUG( node_->get_logger(), "Controllers switched successfully: %s for plugin: %s",
                        message.c_str(), plugin_name_.c_str() );
        } else {
          RCLCPP_ERROR( node_->get_logger(), "Failed to switch controllers: %s for plugin: %s",
                        message.c_str(), plugin_name_.c_str() );
        }
      } );
}

} // namespace hector_gamepad_manager_plugins