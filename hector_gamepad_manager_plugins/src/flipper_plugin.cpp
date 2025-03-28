#include "hector_gamepad_manager_plugins/flipper_plugin.hpp"

namespace hector_gamepad_manager_plugins
{

void FlipperPlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;

  const std::string plugin_namespace = getPluginName();

  node_->declare_parameters<double>( plugin_namespace, {
                                                           { "speed", 1.5 },
                                                           { "flipper_front_factor", 1.0 },
                                                           { "flipper_back_factor", 1.0 },
                                                       } );
  node_->declare_parameters<std::string>(
      plugin_namespace, { { "standard_controller", "flipper_trajectory_controller" },
                          { "command_topic", "flipper_velocity_controller/commands" } } );
  node_->declare_parameters<std::vector<std::string>>(
      plugin_namespace,
      {
          { "teleop_controller",
            { "self_collision_avoidance_controller", "flipper_velocity_controller" } },
      } );

  speed_ = node_->get_parameter( plugin_namespace + ".speed" ).as_double();
  flipper_front_factor_ =
      node_->get_parameter( plugin_namespace + ".flipper_front_factor" ).as_double();
  flipper_back_factor_ =
      node_->get_parameter( plugin_namespace + ".flipper_back_factor" ).as_double();
  standard_controller_ =
      node_->get_parameter( plugin_namespace + ".standard_controller" ).as_string();
  teleop_controller_ =
      node_->get_parameter( plugin_namespace + ".teleop_controller" ).as_string_array();

  std::string command_topic = node_->get_parameter( plugin_namespace + ".command_topic" ).as_string();

  param_cb_handler_ = node->add_on_set_parameters_callback(
      std::bind( &FlipperPlugin::setParamsCb, this, std::placeholders::_1 ) );

  flipper_command_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/" + node_->get_parameter( "robot_namespace" ).as_string() + "/" + command_topic, 10 );

  controller_helper_.initialize( node, plugin_namespace );
  active_ = true;
}

rcl_interfaces::msg::SetParametersResult
FlipperPlugin::setParamsCb( const std::vector<rclcpp::Parameter> &parameters )
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  const auto plugin_namespace = getPluginName();

  for ( const auto &parameter : parameters ) {
    if ( parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
         parameter.get_name().find( plugin_namespace ) != std::string::npos ) {
      const double val = parameter.as_double();
      if ( val <= 0 ) {
        result.successful = false;
        result.reason = "Can't set a negative or null speed for: " + parameter.get_name();
        break;
      }

      if ( parameter.get_name() == plugin_namespace + ".speed" )
        speed_ = val;

      if ( parameter.get_name() == plugin_namespace + ".flipper_front_factor" )
        flipper_front_factor_ = val;

      if ( parameter.get_name() == plugin_namespace + ".flipper_back_factor" )
        flipper_back_factor_ = val;
    }
  }

  return result;
}

std::string FlipperPlugin::getPluginName() { return "flipper_plugin"; }

void FlipperPlugin::handlePress( const std::string &function )
{
  // Activate respective indidual mode only if other is inactive
  if ( function == "individual_front_flipper_control_mode" ) {
    individual_front_flipper_mode_ = !individual_back_flipper_mode_;
    return;
  }

  if ( function == "individual_back_flipper_control_mode" ) {
    individual_back_flipper_mode_ = !individual_front_flipper_mode_;
    return;
  }

  if ( individual_front_flipper_mode_ || individual_back_flipper_mode_ )
    handleIndividualFlipperControlInput( 1, true, function );
  else
    handleBasicControlInput( 1, true, function );
}

void FlipperPlugin::handleRelease( const std::string &function )
{
  // Activate respective indidual mode only if other is inactive
  if ( function == "individual_front_flipper_control_mode" ) {
    // Stop flipper before disable of mode
    handleIndividualFlipperControlInput( 0, true, "flipper_front_up" );
    handleIndividualFlipperControlInput( 0, true, "flipper_back_up" );
    individual_front_flipper_mode_ = false;
    return;
  }

  if ( function == "individual_back_flipper_control_mode" ) {
    // Stop flipper before disable
    handleIndividualFlipperControlInput( 0, true, "flipper_front_up" );
    handleIndividualFlipperControlInput( 0, true, "flipper_back_up" );
    individual_back_flipper_mode_ = false;
    return;
  }

  if ( individual_front_flipper_mode_ || individual_back_flipper_mode_ )
    handleIndividualFlipperControlInput( 0, true, function );
  else
    handleBasicControlInput( 0, true, function );
}

void FlipperPlugin::handleAxis( const std::string &function, const double value )
{
  if ( individual_front_flipper_mode_ || individual_back_flipper_mode_ )
    handleIndividualFlipperControlInput( -1 * value, false, function );
  else
    handleBasicControlInput( -1 * value, false, function );
}

void FlipperPlugin::update()
{
  if ( !active_ )
    return;

  vel_commands_[0] = axis_vel_commands_[0] + button_vel_commands_[0];
  vel_commands_[1] = axis_vel_commands_[1] + button_vel_commands_[1];
  vel_commands_[2] = axis_vel_commands_[2] + button_vel_commands_[2];
  vel_commands_[3] = axis_vel_commands_[3] + button_vel_commands_[3];

  const bool current_cmd_zero = checkCurrentCmdIsZero();

  if ( last_cmd_zero_ && !current_cmd_zero )
    controller_helper_.switchControllers( teleop_controller_, { standard_controller_ } );

  if ( !( last_cmd_zero_ && current_cmd_zero ) )
    publishCommands();
  last_cmd_zero_ = current_cmd_zero;
}

void FlipperPlugin::activate()
{
  active_ = true;
  last_cmd_zero_ = false;
}

void FlipperPlugin::deactivate()
{
  active_ = false;

  // Publish null velocity command
  resetCommands();
  publishCommands();
}

void FlipperPlugin::handleBasicControlInput( const double base_speed_factor, const bool is_button,
                                             const std::string &function )
{
  if ( !active_ )
    return;

  double vel = speed_ * base_speed_factor;

  if ( function == "flipper_back_up" || function == "flipper_back_down" ) {
    vel *= flipper_back_factor_;

    if ( is_button ) {
      button_vel_commands_[2] = vel;
      button_vel_commands_[3] = vel;
    } else {
      axis_vel_commands_[2] = vel;
      axis_vel_commands_[3] = vel;
    }
  }

  if ( function == "flipper_front_up" || function == "flipper_front_down" ) {
    vel *= flipper_front_factor_;

    if ( is_button ) {
      button_vel_commands_[0] = vel;
      button_vel_commands_[1] = vel;
    } else {
      axis_vel_commands_[0] = vel;
      axis_vel_commands_[1] = vel;
    }
  }
}

void FlipperPlugin::handleIndividualFlipperControlInput( const double base_speed_factor,
                                                         const bool is_button,
                                                         const std::string &function )
{
  if ( !active_ )
    return;

  double vel = speed_ * base_speed_factor;

  // Right side
  if ( function == "flipper_front_up" || function == "flipper_front_down" ) {

    if ( is_button ) {

      if ( individual_front_flipper_mode_ )
        button_vel_commands_[1] = vel * flipper_front_factor_;
      else
        button_vel_commands_[3] = vel * flipper_back_factor_;

    } else {

      if ( individual_front_flipper_mode_ )
        axis_vel_commands_[1] = vel * flipper_front_factor_;
      else
        axis_vel_commands_[3] = vel * flipper_back_factor_;
    }
  }

  // Left side
  if ( function == "flipper_back_up" || function == "flipper_back_down" ) {

    if ( is_button ) {

      if ( individual_front_flipper_mode_ )
        button_vel_commands_[0] = vel * flipper_front_factor_;
      else
        button_vel_commands_[2] = vel * flipper_back_factor_;

    } else {

      if ( individual_front_flipper_mode_ )
        axis_vel_commands_[0] = vel * flipper_front_factor_;
      else
        axis_vel_commands_[2] = vel * flipper_back_factor_;
    }
  }
}

void FlipperPlugin::resetCommands()
{
  vel_commands_[0] = 0.0;
  vel_commands_[1] = 0.0;
  vel_commands_[2] = 0.0;
  vel_commands_[3] = 0.0;
}

void FlipperPlugin::publishCommands() const
{
  std_msgs::msg::Float64MultiArray cmd_msg;
  cmd_msg.data.assign( vel_commands_.begin(), vel_commands_.end() );
  flipper_command_publisher_->publish( cmd_msg );
}

bool FlipperPlugin::checkCurrentCmdIsZero() const
{
  for ( const double cmd : vel_commands_ ) {
    if ( cmd != 0.0 )
      return false;
  }
  return true;
}

} // namespace hector_gamepad_manager_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_gamepad_manager_plugins::FlipperPlugin,
                        hector_gamepad_manager_interface::GamepadFunctionPlugin )