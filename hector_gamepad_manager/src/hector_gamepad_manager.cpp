#include "hector_gamepad_manager/hector_gamepad_manager.hpp"

namespace hector_gamepad_manager
{
HectorGamepadManager::HectorGamepadManager( const rclcpp::Node::SharedPtr &node )
    : node_( node ),
      plugin_loader_( "hector_gamepad_manager", "hector_gamepad_manager::GamepadFunctionPlugin" )
{
  const std::string package_path =
      ament_index_cpp::get_package_share_directory( "hector_gamepad_manager" );

  node_->declare_parameter<std::string>( "config_filename", "gamepad_mappings.yaml" );
  const std::string config_filename = node_->get_parameter( "config_filename" ).as_string();

  if ( !loadConfig( package_path + "/config/" + config_filename ) ) {
    RCLCPP_ERROR( node_->get_logger(), "Failed to load config file %s", config_filename.c_str() );
    return;
  }

  joy_subscription_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind( &HectorGamepadManager::joyCallback, this, std::placeholders::_1 ) );
}

bool HectorGamepadManager::loadConfig( const std::string &file_path )
{
  try {
    YAML::Node config = YAML::LoadFile( file_path );

    initMappings( config, "axes", axis_mappings_ );
    initMappings( config, "buttons", button_mappings_ );

    return true;
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( node_->get_logger(), "Error loading YAML file: %s", e.what() );
    return false;
  }
}

void HectorGamepadManager::initMappings( const YAML::Node &config, const std::string &type,
                                         std::unordered_map<int, ActionMapping> &mappings )
{
  if ( config[type] ) {
    for ( const auto &entry : config[type] ) {
      int id = entry.first.as<int>();
      YAML::Node mapping = entry.second;
      std::string plugin = mapping["plugin"].as<std::string>();
      std::string function = mapping["function"].as<std::string>();

      if ( !plugin.empty() && !function.empty() ) {
        mappings[id] = { plugin, function };
      }
    }
  }

  // Load plugins
  for ( const auto &mapping : mappings ) { loadPlugin( mapping.second.plugin_name ); }
}

void HectorGamepadManager::loadPlugin( const std::string &plugin_name )
{
  if ( plugins_.count( plugin_name ) == 0 ) {
    try {
      RCLCPP_INFO( node_->get_logger(), "Trying to load plugin: %s", plugin_name.c_str() );
      std::shared_ptr<GamepadFunctionPlugin> plugin =
          plugin_loader_.createSharedInstance( plugin_name );
      plugin->initialize( node_, true );
      plugins_[plugin_name] = plugin;
      RCLCPP_INFO( node_->get_logger(), "Loaded plugin: %s", plugin_name.c_str() );
    } catch ( const std::exception &e ) {
      RCLCPP_ERROR( node_->get_logger(), "Failed to load plugin %s: %s", plugin_name.c_str(),
                    e.what() );
    }
  }
}

void HectorGamepadManager::joyCallback( const sensor_msgs::msg::Joy::SharedPtr msg )
{
  // Handle buttons
  for ( const auto &button_mapping : button_mappings_ ) {
    const bool pressed = msg->buttons[button_mapping.first];
    const auto &action = button_mapping.second;
    if ( plugins_.count( action.plugin_name ) ) {
      plugins_[action.plugin_name]->handleButton( action.function_name, pressed );
    } else {
      RCLCPP_ERROR( node_->get_logger(), "Plugin not found: %s", action.plugin_name.c_str() );
    }
  }

  // Handle axes
  for ( const auto &axis_mapping : axis_mappings_ ) {
    const double value = msg->axes[axis_mapping.first];
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
} // namespace hector_gamepad_manager
