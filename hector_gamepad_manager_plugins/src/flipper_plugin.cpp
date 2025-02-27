#include "hector_gamepad_manager_plugins/flipper_plugin.hpp"

namespace hector_gamepad_manager_plugins
{

void FlipperPlugin::initialize( const rclcpp::Node::SharedPtr &node)
{
  node_ = node;

  std::string plugin_namespace = getPluginName();

  node_->declare_parameters<double>( plugin_namespace, {{ "speed", 1.5 },
                                                         { "flipper_front_factor", 1.0 },
                                                         { "flipper_back_factor", 1.0 },
                                                         });
  node_->declare_parameters<std::string>(plugin_namespace, {{ "standard_controller", "flipper_controller" },
                                                         { "teleop_controller", "flipper_controller_teleop" },
                                                         });

  speed_ = node_->get_parameter( plugin_namespace + ".speed" ).as_double();
  flipper_front_factor_ = node_->get_parameter( plugin_namespace + ".flipper_front_factor" ).as_double();
  flipper_back_factor_ = node_->get_parameter( plugin_namespace + ".flipper_back_factor" ).as_double();
  standard_controller_ = node_->get_parameter( plugin_namespace + ".standard_controller" ).as_string();
  teleop_controller_ = node_->get_parameter( plugin_namespace + ".teleop_controller" ).as_string();

  // ros controller mapping [fr_l, fr_r, b_l, b_r]
  vel_commands_.insert(vel_commands_.begin(), {0.0, 0.0, 0.0, 0.0});

  flipper_command_publisher_ =
      node_->create_publisher<std_msgs::msg::Float64MultiArray>("/" + node_->get_parameter("robot_namespace").as_string() + "/flipper_controller_teleop/commands", 10 );
  
  controller_helper_.initialize(node, plugin_namespace);
  RCLCPP_INFO( node_->get_logger(), "Attempting controller switch");

  active_=true;
    
}

std::string FlipperPlugin::getPluginName(){
  return "flipper_plugin";
}

void FlipperPlugin::handlePress( const std::string &function)
{

  if ( !active_ ) 
    return;

  if(function == "flipper_back_up") 
      set_back_flipper_command(speed_ * flipper_back_factor_);

  if(function == "flipper_front_up")
      set_front_flipper_command(speed_ * flipper_front_factor_);

}

void FlipperPlugin::handleHold( const std::string &function)
{

  if ( !active_ ) 
    return;

  if(function == "flipper_back_up") 
      set_back_flipper_command(speed_ * flipper_back_factor_);

  if(function == "flipper_front_up")
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
  if (!active_) 
    return;

  bool current_cmd_zero = check_current_cmd_is_zero();

  if(last_cmd_zero_ && !current_cmd_zero)
    controller_helper_.switchControllers({teleop_controller_}, {standard_controller_}); 
    

  if(!(last_cmd_zero_ && current_cmd_zero))
    publish_commands();
  reset_commands();
  last_cmd_zero_ = current_cmd_zero;
}

void FlipperPlugin::activate() {
  //controller_helper_.switchControllers({teleop_controller_}, {standard_controller_}); 
  active_ = true;
  last_cmd_zero_=false;
}

void FlipperPlugin::deactivate()
{
  active_ = false;

  //Publish null velocity command
  reset_commands();
  publish_commands();

  //controller_helper_.switchControllers({standard_controller_}, {teleop_controller_}); 
} 

void FlipperPlugin::set_front_flipper_command(double vel){
  // To avoid ovveride of trigger cmd from bumber zero_val add vel
  // Vel commands are reset after every update call
  vel_commands_[0] += vel;
  vel_commands_[1] += vel;
}

void FlipperPlugin::set_back_flipper_command(double vel){
  // To avoid ovveride of trigger cmd from bumber zero_val add vel
  // Vel commands are reset after every update call
  vel_commands_[2] += vel;
  vel_commands_[3] += vel;
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

bool FlipperPlugin::check_current_cmd_is_zero(){
 for(double cmd : vel_commands_){
   if(cmd != 0.0)
     return false;
 }
 return true;
}


}// namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::FlipperPlugin,
                        hector_gamepad_manager::GamepadFunctionPlugin )