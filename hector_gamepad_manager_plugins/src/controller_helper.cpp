#include "hector_gamepad_manager_plugins/controller_helper.hpp"
#include <controller_orchestrator/controller_orchestrator.hpp>

namespace hector_gamepad_manager_plugins
{

void ControllerHelper::initialize( const rclcpp::Node::SharedPtr &node, std::string plugin_name )
{

  node_ = node;

  controller_orchestrator_ = std::make_shared<controller_orchestrator::ControllerOrchestrator>(
      node_, "/" + node_->get_parameter( "robot_namespace" ).as_string() + "/controller_manager" );
}

void ControllerHelper::switchControllers( std::vector<std::string> start_controllers )
{
  if ( start_controllers.empty() )
    return;

  (void)controller_orchestrator_->smartSwitchController( start_controllers );
}

} // namespace hector_gamepad_manager_plugins