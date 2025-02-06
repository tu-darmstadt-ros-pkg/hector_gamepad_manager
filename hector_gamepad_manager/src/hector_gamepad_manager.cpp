#include "hector_gamepad_manager/hector_gamepad_manager.hpp"

namespace hector_gamepad_manager
{
HectorGamepadManager::HectorGamepadManager( const rclcpp::Node::SharedPtr &node )
    : node_( node ),
      plugin_loader_( "hector_gamepad_manager", "hector_gamepad_manager::GamepadFunctionPlugin" ),
      first_config_( true )
{
  const std::string package_path =
      ament_index_cpp::get_package_share_directory( "hector_gamepad_manager" );

  node_->declare_parameter<std::vector<std::string>>( "config_filenames",
                                                      { "gamepad_mappings.yaml" } );
  const std::vector<std::string> config_filenames =
      node_->get_parameter( "config_filenames" ).as_string_array();

  for ( const auto &config_filename : config_filenames ) {
    if ( !loadConfig( package_path + "/config/" + config_filename ) ) {
      RCLCPP_ERROR( node_->get_logger(), "Failed to load config file %s", config_filename.c_str() );
      return;
    }
  }

  inputs_ = GamepadInputs();

  joy_subscription_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 1, std::bind( &HectorGamepadManager::joyCallback, this, std::placeholders::_1 ) );
}

bool HectorGamepadManager::loadConfig( const std::string &file_path )
{
  try {
    const YAML::Node config = YAML::LoadFile( file_path );

    // Get the name of the file without the path and extension
    const std::string filename =
        file_path.substr( file_path.find_last_of( '/' ) + 1 ).substr( 0, file_path.find_last_of( '.' ) );

    // Add empty mappings for the filename
    button_mappings_[filename] = {};
    axis_mappings_[filename] = {};

    if ( first_config_ ) {
      active_config_ = filename;
      active_button_mappings_ = &button_mappings_[filename];
      active_axis_mappings_ = &axis_mappings_[filename];
    }

    if ( !initMappings( config, "buttons", button_mappings_[filename] ) ||
         !initMappings( config, "axes", axis_mappings_[filename] ) ) {
      return false;
    }

    first_config_ = false;

    return true;
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( node_->get_logger(), "Error loading YAML file: %s", e.what() );
    return false;
  }
}

bool HectorGamepadManager::initMappings( const YAML::Node &config, const std::string &type,
                                         std::unordered_map<int, FunctionMapping> &mappings )
{
  if ( config[type] ) {
    for ( const auto &entry : config[type] ) {
      int id = entry.first.as<int>();
      YAML::Node mapping = entry.second;
      auto plugin = mapping["plugin"].as<std::string>();
      auto function = mapping["function"].as<std::string>();

      if ( !plugin.empty() && !function.empty() ) {
        mappings[id] = { plugin, function };
      }
    }
  } else {
    RCLCPP_ERROR( node_->get_logger(), "No %s found in config file", type.c_str() );
    return false;
  }

  // Load plugins
  for ( const auto &mapping : mappings ) {
    const std::string plugin_name = mapping.second.plugin_name;
    if ( plugins_.count( plugin_name ) == 0 ) {
      try {
        std::shared_ptr<GamepadFunctionPlugin> plugin =
            plugin_loader_.createSharedInstance( plugin_name );
        plugin->initialize( node_, first_config_ );
        plugins_[plugin_name] = plugin;
        RCLCPP_INFO( node_->get_logger(), "Loaded plugin: %s", plugin_name.c_str() );
      } catch ( const std::exception &e ) {
        RCLCPP_ERROR( node_->get_logger(), "Failed to load plugin %s: %s", plugin_name.c_str(),
                      e.what() );
        return false;
      }
    }
  }

  return true;
}

void HectorGamepadManager::joyCallback( const sensor_msgs::msg::Joy::SharedPtr msg )
{
  convert_joy_to_gamepad_inputs( msg );

  // Handle buttons
  for ( const auto &button_mapping : *active_button_mappings_ ) {
    const bool pressed = inputs_.buttons[button_mapping.first];
    const auto &action = button_mapping.second;
    if ( plugins_.count( action.plugin_name ) ) {
      plugins_[action.plugin_name]->handleButton( action.function_name, pressed );
    } else {
      RCLCPP_ERROR( node_->get_logger(), "Plugin not found: %s", action.plugin_name.c_str() );
    }
  }

  // Handle axes
  for ( const auto &axis_mapping : *active_axis_mappings_ ) {
    const float value = inputs_.axes[axis_mapping.first];
    const auto &action = axis_mapping.second;
    if ( plugins_.count( action.plugin_name ) ) {
      plugins_[action.plugin_name]->handleAxis( action.function_name, value );
    } else {
      RCLCPP_ERROR( node_->get_logger(), "Plugin not found: %s", action.plugin_name.c_str() );
    }
  }

  // Update all plugins
  for ( const auto &plugin : plugins_ ) { plugin.second->update(); }
}

void HectorGamepadManager::convert_joy_to_gamepad_inputs( const sensor_msgs::msg::Joy::SharedPtr &msg )
{
  // Axes
  inputs_.axes[0] = msg->axes[0];                    // Left joystick left/right
  inputs_.axes[1] = msg->axes[1];                    // Left joystick up/down
  inputs_.axes[2] = -0.5f * ( msg->axes[2] - 1.0f ); // LT: Change range from [1, -1] to [0, 1]
  inputs_.axes[3] = msg->axes[3];                    // Right joystick left/right
  inputs_.axes[4] = msg->axes[4];                    // Right joystick up/down
  inputs_.axes[5] = -0.5f * ( msg->axes[5] - 1.0f ); // RT: Change range from [1, -1] to [0, 1]
  inputs_.axes[6] = msg->axes[6];                    // Cross left/right
  inputs_.axes[7] = msg->axes[7];                    // Cross up/down

  // Buttons
  inputs_.buttons[0] = msg->buttons[0];                   // Button A
  inputs_.buttons[1] = msg->buttons[1];                   // Button B
  inputs_.buttons[2] = msg->buttons[2];                   // Button X
  inputs_.buttons[3] = msg->buttons[3];                   // Button Y
  inputs_.buttons[4] = msg->buttons[4];                   // Button LB
  inputs_.buttons[5] = msg->buttons[5];                   // Button RB
  inputs_.buttons[6] = msg->buttons[6];                   // Button Back
  inputs_.buttons[7] = msg->buttons[7];                   // Button Start
  inputs_.buttons[8] = msg->buttons[8];                   // Button Guide
  inputs_.buttons[9] = msg->buttons[9];                   // Left joystick pressed
  inputs_.buttons[10] = msg->buttons[10];                 // Right joystick pressed
  inputs_.buttons[11] = inputs_.axes[0] > AXIS_DEADZONE;  // Left joystick left
  inputs_.buttons[12] = inputs_.axes[0] < -AXIS_DEADZONE; // Left joystick right
  inputs_.buttons[13] = inputs_.axes[1] > AXIS_DEADZONE;  // Left joystick up
  inputs_.buttons[14] = inputs_.axes[1] < -AXIS_DEADZONE; // Left joystick down
  inputs_.buttons[15] = inputs_.axes[2] > AXIS_DEADZONE;  // LT button
  inputs_.buttons[16] = inputs_.axes[3] > AXIS_DEADZONE;  // Right joystick left
  inputs_.buttons[17] = inputs_.axes[3] < -AXIS_DEADZONE; // Right joystick right
  inputs_.buttons[18] = inputs_.axes[4] > AXIS_DEADZONE;  // Right joystick up
  inputs_.buttons[19] = inputs_.axes[4] < -AXIS_DEADZONE; // Right joystick down
  inputs_.buttons[20] = inputs_.axes[5] > AXIS_DEADZONE;  // RT button
  inputs_.buttons[21] = inputs_.axes[6] == 1.0f;          // Cross left
  inputs_.buttons[22] = inputs_.axes[6] == -1.0f;         // Cross right
  inputs_.buttons[23] = inputs_.axes[7] == 1.0f;          // Cross up
  inputs_.buttons[24] = inputs_.axes[7] == -1.0f;         // Cross down
}
} // namespace hector_gamepad_manager
