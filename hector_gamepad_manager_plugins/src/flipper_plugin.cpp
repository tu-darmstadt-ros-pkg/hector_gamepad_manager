#include "hector_gamepad_manager_plugins/flipper_plugin.hpp"

namespace hector_gamepad_manager_plugins
{

void FlipperPlugin::initialize( const rclcpp::Node::SharedPtr &node, const bool active )
{
  node_ = node;
  active_ = active;

  plugin_namespace_ = "flipper_plugin";

  node_->declare_parameters<double>( plugin_namespace_, {{ "speed", 1.5 },
                                                         { "flipper_front_factor", 1.0 },
                                                         { "flipper_back_factor", 1.0 }});

  speed_ = node_->get_parameter( plugin_namespace_ + ".speed" ).as_double();
  flipper_front_factor_ = node_->get_parameter( plugin_namespace_ + ".flipper_front_factor" ).as_double();
  flipper_back_factor_ = node_->get_parameter( plugin_namespace_ + ".flipper_back_factor" ).as_double();

  // ros controller mapping [fr_l, fr_r, b_l, b_r]
  vel_commands_.insert(vel_commands_.begin(), {0.0, 0.0, 0.0, 0.0});

  flipper_command_publisher_ =
      node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_controller_teleop/commands", 10 );
}

void FlipperPlugin::handleButton( const std::string &function, const bool pressed )
{
  if ( !active_ ) 
    return;

  RCLCPP_INFO( node_->get_logger(), "Function: %s, Pressed: %d", function.c_str(), pressed);

  if(function == "flipper_back_up" && pressed) 
      set_back_flipper_command(speed_ * flipper_back_factor_);

  if(function == "flipper_front_up" && pressed)
      set_front_flipper_command(speed_ * flipper_front_factor_);

}

void FlipperPlugin::handleAxis( const std::string &function, const double value )
{
  if ( !active_ )
    return;

  if(function == "flipper_back_down")
    set_back_flipper_command(- speed_ * flipper_back_factor_ * value);

  if(function == "flipper_front_down")
    set_front_flipper_command(- speed_ * flipper_front_factor_ * value);
  
}

void FlipperPlugin::update()
{
  if ( !active_ ) 
    return;  

  publish_commands();
  reset_commands();
}

void FlipperPlugin::activate() { active_ = true;  }

void FlipperPlugin::deactivate()
{
  active_ = false;

  reset_commands();
  //Publish null velocity command
  publish_commands();
} 

void FlipperPlugin::set_front_flipper_command(double vel){
  vel_commands_[0] = vel;
  vel_commands_[1] = vel;
}

void FlipperPlugin::set_back_flipper_command(double vel){
  vel_commands_[2] = vel;
  vel_commands_[3] = vel;
}

void FlipperPlugin::reset_commands(){
  vel_commands_[0] = 0.0;
  vel_commands_[1] = 0.0;
  vel_commands_[2] = 0.0;
  vel_commands_[3] = 0.0;
}

void FlipperPlugin::publish_commands(){
  std_msgs::msg::Float64MultiArray cmd_msg;
  cmd_msg.data = vel_commands_;
  flipper_command_publisher_->publish(cmd_msg);
}

}// namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::FlipperPlugin,
                        hector_gamepad_manager::GamepadFunctionPlugin )