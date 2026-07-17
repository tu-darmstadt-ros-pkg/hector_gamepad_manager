#include "hector_gamepad_manager/gamepad_mapping_builder.hpp"

#include <algorithm>
#include <vector>

namespace hector_gamepad_manager
{
namespace
{
using hector_gamepad_manager_msgs::msg::GamepadAction;

// Append an action to the list if it is configured (has a function).
void addAction( std::vector<GamepadAction> &actions, uint8_t event, const ActionMapping &mapping )
{
  if ( mapping.empty() )
    return;
  GamepadAction action;
  action.event = event;
  action.function = mapping.function;
  action.description = mapping.description;
  actions.push_back( action );
}

std::string pluginId( const std::shared_ptr<GamepadFunctionPlugin> &plugin )
{
  return plugin ? plugin->getPluginId() : std::string();
}
} // namespace

hector_gamepad_manager_msgs::msg::GamepadMapping
buildGamepadMappingMsg( const std::map<std::string, GamepadConfig> &configs,
                        const std::array<ConfigSwitch, kNumButtons> &config_switches,
                        const std::string &default_config )
{
  hector_gamepad_manager_msgs::msg::GamepadMapping msg;
  msg.default_config = default_config;

  for ( const auto &[config_name, config] : configs ) {
    hector_gamepad_manager_msgs::msg::GamepadConfigMapping config_msg;
    config_msg.name = config_name;
    config_msg.description = config.description;

    for ( const auto &[index, mapping] : config.button_mappings ) {
      hector_gamepad_manager_msgs::msg::GamepadButtonMapping button_msg;
      button_msg.index = static_cast<uint8_t>( index );
      button_msg.plugin = pluginId( mapping.plugin );
      addAction( button_msg.actions, GamepadAction::EVENT_PRESS, mapping.on_press );
      addAction( button_msg.actions, GamepadAction::EVENT_DOUBLE_PRESS, mapping.on_double_press );
      addAction( button_msg.actions, GamepadAction::EVENT_HOLD, mapping.on_hold );
      addAction( button_msg.actions, GamepadAction::EVENT_RELEASE, mapping.on_release );
      config_msg.buttons.push_back( button_msg );
    }
    std::sort( config_msg.buttons.begin(), config_msg.buttons.end(),
               []( const auto &a, const auto &b ) { return a.index < b.index; } );

    for ( const auto &[index, mapping] : config.axis_mappings ) {
      hector_gamepad_manager_msgs::msg::GamepadAxisMapping axis_msg;
      axis_msg.index = static_cast<uint8_t>( index );
      axis_msg.plugin = pluginId( mapping.plugin );
      axis_msg.function = mapping.function_name;
      axis_msg.description = mapping.description;
      config_msg.axes.push_back( axis_msg );
    }
    std::sort( config_msg.axes.begin(), config_msg.axes.end(),
               []( const auto &a, const auto &b ) { return a.index < b.index; } );

    msg.configs.push_back( config_msg );
  }

  for ( size_t i = 0; i < config_switches.size(); ++i ) {
    if ( config_switches[i].config.empty() )
      continue;
    hector_gamepad_manager_msgs::msg::GamepadConfigSwitch switch_msg;
    switch_msg.index = static_cast<uint8_t>( i );
    switch_msg.config = config_switches[i].config;
    switch_msg.description = config_switches[i].description;
    msg.config_switches.push_back( switch_msg );
  }

  return msg;
}
} // namespace hector_gamepad_manager
