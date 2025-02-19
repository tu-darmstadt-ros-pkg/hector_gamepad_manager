#include "hector_gamepad_manager/hector_gamepad_manager.hpp"

namespace hector_gamepad_manager
{
HectorGamepadManager::HectorGamepadManager( const rclcpp::Node::SharedPtr &node )
    : node_( node ),
      plugin_loader_( "hector_gamepad_manager", "hector_gamepad_manager::GamepadFunctionPlugin" )
{

  node_->declare_parameter<std::string>( "config_switches_filename", "config_switches" );
  const std::string config_switches_filename =
      node_->get_parameter( "config_switches_filename" ).as_string();
  node_->declare_parameter<std::string>( "robot_name", "athena" );
  robot_name_ = node_->get_parameter( "robot_name" ).as_string();
  rclcpp::QoS qos_profile( 1 ); // Keep only the last message
  qos_profile.reliability( RMW_QOS_POLICY_RELIABILITY_RELIABLE );
  qos_profile.durability( RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL );
  active_config_publisher_ =
      node_->create_publisher<std_msgs::msg::String>( "active_config", qos_profile );

  if ( loadConfigSwitchesConfig( config_switches_filename ) ) {
    switchConfig( default_config_ );
    joy_subscription_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, std::bind( &HectorGamepadManager::joyCallback, this, std::placeholders::_1 ) );
  }
}

bool HectorGamepadManager::loadConfigSwitchesConfig( const std::string &file_name )
{

  try {
    const YAML::Node config = YAML::LoadFile( getPath( "hector_gamepad_manager", file_name ) );
    for ( const auto &entry : config["buttons"] ) {
      const int id = entry.first.as<int>();
      YAML::Node mapping = entry.second;
      auto config_name = mapping["config"].as<std::string>();
      auto pkg_name = mapping["package"].as<std::string>();
      if ( config_name.empty() )
        continue; // skip empty mappings

      RCLCPP_INFO( node_->get_logger(), "Loading config file %s", config_name.c_str() );
      if ( !loadConfig( pkg_name, config_name ) ) {
        RCLCPP_ERROR( node_->get_logger(), "Failed to load config file %s", config_name.c_str() );
        return false;
      }
      config_switch_button_mapping_[id] = config_name;
    }
    default_config_ = config["default_config"].as<std::string>();
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( node_->get_logger(), "Error loading Config Switch YAML file: %s", e.what() );
    return false;
  }
  return true;
}

bool HectorGamepadManager::loadConfig( const std::string &pkg_name, const std::string &file_name )
{
  try {
    const YAML::Node config = YAML::LoadFile( getPath( pkg_name, file_name ) );

    // Add empty mappings for the filename
    configs_[file_name] = GamepadConfig();

    if ( !initMappings( config, "buttons", configs_[file_name].button_mappings ) ||
         !initMappings( config, "axes", configs_[file_name].axis_mappings ) ) {
      return false;
    }

    return true;
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( node_->get_logger(), "Error loading YAML file: %s", e.what() );
    return false;
  }
}

bool HectorGamepadManager::switchConfig( const std::string &config_name )
{
  if ( configs_.count( config_name ) == 0 ) {
    RCLCPP_ERROR( node_->get_logger(), "Config %s not found. Cannot switch the gamepad config",
                  config_name.c_str() );
    return false;
  }
  if ( config_name == active_config_ )
    return true;
  RCLCPP_INFO( node_->get_logger(), "Switching from config %s to config: %s",
               active_config_.c_str(), config_name.c_str() );
  deactivatePlugins();
  active_config_publisher_->publish( std_msgs::msg::String().set__data( config_name ) );
  active_config_ = config_name;
  activatePlugins( config_name );
  return true;
}

bool HectorGamepadManager::initMappings( const YAML::Node &config, const std::string &type,
                                         std::unordered_map<int, FunctionMapping> &mappings )
{
  if ( config[type] ) {
    for ( const auto &entry : config[type] ) {
      int id = entry.first.as<int>();
      YAML::Node mapping = entry.second;
      auto plugin_name = mapping["plugin"].as<std::string>();
      auto function = mapping["function"].as<std::string>();

      if ( !plugin_name.empty() && !function.empty() ) {

        // make sure plugin is loaded
        if ( plugins_.count( plugin_name ) == 0 ) {
          try {
            std::shared_ptr<GamepadFunctionPlugin> plugin =
                plugin_loader_.createSharedInstance( plugin_name );
            plugin->initialize( node_, robot_name_ );
            plugins_[plugin_name] = plugin;
            RCLCPP_INFO( node_->get_logger(), "Loaded plugin: %s", plugin_name.c_str() );
          } catch ( const std::exception &e ) {
            RCLCPP_ERROR( node_->get_logger(), "Failed to load plugin %s: %s", plugin_name.c_str(),
                          e.what() );
            return false;
          }
        }
        mappings[id] = { plugins_[plugin_name], function };
      }
    }
  } else {
    RCLCPP_ERROR( node_->get_logger(), "No %s found in config file", type.c_str() );
    return false;
  }
  return true;
}

bool HectorGamepadManager::handleConfigurationSwitches( const GamepadInputs &inputs )
{

  if ( inputs.buttons[CONFIG_SWITCH_BUTTON] ) {
    // test if and only if one additional button is pressed
    int count = 0;
    std::string new_config;
    for ( size_t i = 0; i < inputs.buttons.size(); i++ ) {
      if ( inputs.buttons[i] && i != CONFIG_SWITCH_BUTTON ) {
        count++;
        new_config = config_switch_button_mapping_[i];
      }
    }
    if ( count == 1 && !new_config.empty() ) {
      switchConfig( new_config );
    }
    return true;
  }
  return false;
}

void HectorGamepadManager::joyCallback( const sensor_msgs::msg::Joy::SharedPtr msg )
{
  const auto inputs = convertJoyToGamepadInputs( msg );
  // ignore normal button / axis behavior if configuration switching is in progress
  if ( handleConfigurationSwitches( inputs ) )
    return;

  // Handle buttons
  for ( const auto &button_mapping : configs_[active_config_].button_mappings ) {
    const bool pressed = inputs.buttons[button_mapping.first];
    const auto &action = button_mapping.second;
    button_mapping.second.plugin->handleButton( action.function_name, pressed );
  }

  // Handle axes
  for ( const auto &axis_mapping : configs_[active_config_].axis_mappings ) {
    const float value = inputs.axes[axis_mapping.first];
    const auto &action = axis_mapping.second;
    axis_mapping.second.plugin->handleAxis( action.function_name, value );
  }

  // Update all active plugins
  for ( const auto &plugin : active_plugins_ ) { plugin->update(); }
}

void HectorGamepadManager::activatePlugins( const std::string &config_name )
{
  // activate all  plugins present in the button_mappings_ and axis_mappings_ of the given config
  if ( configs_.count( config_name ) == 0 ) {
    RCLCPP_ERROR( node_->get_logger(), "Config %s not found. Cannot activate the gamepad config",
                  config_name.c_str() );
    return;
  }
  // activate all plugins present in the button_mappings_
  for ( const auto &button_mapping : configs_[config_name].button_mappings ) {
    if ( !button_mapping.second.plugin->isActive() ) {
      button_mapping.second.plugin->activate();
      active_plugins_.push_back( button_mapping.second.plugin );
    }
  }
  // activate all plugins present in the axis_mappings_
  for ( const auto &axis_mapping : configs_[config_name].axis_mappings ) {
    if ( !axis_mapping.second.plugin->isActive() ) {
      axis_mapping.second.plugin->activate();
      active_plugins_.push_back( axis_mapping.second.plugin );
    }
  }
}

void HectorGamepadManager::deactivatePlugins()
{
  for ( const auto &plugin : plugins_ ) {
    if ( plugin.second->isActive() ) {
      plugin.second->deactivate();
      RCLCPP_INFO( node_->get_logger(), "Deactivated plugin: %s", plugin.first.c_str() );
    }
  }
  active_plugins_.clear();
}

HectorGamepadManager::GamepadInputs
HectorGamepadManager::convertJoyToGamepadInputs( const sensor_msgs::msg::Joy::SharedPtr &msg )
{
  GamepadInputs inputs;
  // Axes
  inputs.axes[0] = msg->axes[0];                    // Left joystick left/right
  inputs.axes[1] = msg->axes[1];                    // Left joystick up/down
  inputs.axes[2] = -0.5f * ( msg->axes[2] - 1.0f ); // LT: Change range from [1, -1] to [0, 1]
  inputs.axes[3] = msg->axes[3];                    // Right joystick left/right
  inputs.axes[4] = msg->axes[4];                    // Right joystick up/down
  inputs.axes[5] = -0.5f * ( msg->axes[5] - 1.0f ); // RT: Change range from [1, -1] to [0, 1]
  inputs.axes[6] = msg->axes[6];                    // Cross left/right
  inputs.axes[7] = msg->axes[7];                    // Cross up/down

  // Buttons
  inputs.buttons[0] = msg->buttons[0];   // Button A
  inputs.buttons[1] = msg->buttons[1];   // Button B
  inputs.buttons[2] = msg->buttons[2];   // Button X
  inputs.buttons[3] = msg->buttons[3];   // Button Y
  inputs.buttons[4] = msg->buttons[4];   // Button LB
  inputs.buttons[5] = msg->buttons[5];   // Button RB
  inputs.buttons[6] = msg->buttons[6];   // Button Back
  inputs.buttons[7] = msg->buttons[7];   // Button Start
  inputs.buttons[8] = msg->buttons[8];   // Button Guide -> Reserved for config switches
  inputs.buttons[9] = msg->buttons[9];   // Left joystick pressed
  inputs.buttons[10] = msg->buttons[10]; // Right joystick pressed
  inputs.buttons[11] = inputs.axes[0] > AXIS_DEADZONE;  // Left joystick left
  inputs.buttons[12] = inputs.axes[0] < -AXIS_DEADZONE; // Left joystick right
  inputs.buttons[13] = inputs.axes[1] > AXIS_DEADZONE;  // Left joystick up
  inputs.buttons[14] = inputs.axes[1] < -AXIS_DEADZONE; // Left joystick down
  inputs.buttons[15] = inputs.axes[2] > AXIS_DEADZONE;  // LT button
  inputs.buttons[16] = inputs.axes[3] > AXIS_DEADZONE;  // Right joystick left
  inputs.buttons[17] = inputs.axes[3] < -AXIS_DEADZONE; // Right joystick right
  inputs.buttons[18] = inputs.axes[4] > AXIS_DEADZONE;  // Right joystick up
  inputs.buttons[19] = inputs.axes[4] < -AXIS_DEADZONE; // Right joystick down
  inputs.buttons[20] = inputs.axes[5] > AXIS_DEADZONE;  // RT button
  inputs.buttons[21] = inputs.axes[6] == 1.0f;          // Cross left
  inputs.buttons[22] = inputs.axes[6] == -1.0f;         // Cross right
  inputs.buttons[23] = inputs.axes[7] == 1.0f;          // Cross up
  inputs.buttons[24] = inputs.axes[7] == -1.0f;         // Cross down
  return inputs;
}

std::string HectorGamepadManager::getPath( const std::string &pkg_name, const std::string &file_name )
{
  const auto package_path = ament_index_cpp::get_package_share_directory( pkg_name );
  return package_path + "/config/" + file_name + ".yaml";
}
} // namespace hector_gamepad_manager
