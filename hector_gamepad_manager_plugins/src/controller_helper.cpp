#include "hector_gamepad_manager_plugins/controller_helper.hpp"
#include <controller_orchestrator/controller_orchestrator.hpp>

namespace hector_gamepad_manager_plugins
{

void ControllerHelper::initialize( const rclcpp::Node::SharedPtr &node,
                                   const std::string &plugin_name )
{

  node_ = node;
  plugin_name_ = plugin_name;
  const auto robot_ns = node_->get_parameter( "robot_namespace" ).as_string();
  controller_orchestrator_ = std::make_shared<controller_orchestrator::ControllerOrchestrator>(
      node_, "/" + robot_ns + "/controller_manager" );
  // keep track of active controllers
  auto qos = rclcpp::QoS( 1 );
  qos.reliability( RMW_QOS_POLICY_RELIABILITY_RELIABLE );
  qos.durability( RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL );
  ctrl_subscription_ =
      node_->create_subscription<controller_manager_msgs::msg::ControllerManagerActivity>(
          "/" + robot_ns + "/controller_manager/activity", qos,
          [this]( const controller_manager_msgs::msg::ControllerManagerActivity::SharedPtr msg ) {
            active_controllers_.clear();
            for ( const auto &controller : msg->controllers ) {
              if ( controller.state.label == "active" ) {
                active_controllers_.push_back( controller.name );
              }
            }
          } );
}

void ControllerHelper::switchControllers( const std::vector<std::string> &start_controllers ) const
{
  if ( start_controllers.empty() )
    return;
  // check if already active
  bool all_active = true;
  for ( const auto &ctrl : start_controllers ) {
    if ( std::find( active_controllers_.begin(), active_controllers_.end(), ctrl ) ==
         active_controllers_.end() ) {
      all_active = false;
      break;
    }
  }
  if ( all_active )
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