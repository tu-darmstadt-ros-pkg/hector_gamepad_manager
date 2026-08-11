#include "hector_gamepad_manager/gamepad_mapping_builder.hpp"

#include "hector_gamepad_manager/gamepad_buttons.hpp"

#include <algorithm>
#include <vector>

namespace hector_gamepad_manager
{
namespace
{
using hector_gamepad_manager_msgs::msg::GamepadAction;

// Append an action to the list if it is configured (has a non-empty mapping).
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

// Ascending keys of an unordered map, so the emitted order does not depend on its bucket layout.
template<typename Map>
std::vector<int> sortedKeys( const Map &map )
{
  std::vector<int> keys;
  keys.reserve( map.size() );
  for ( const auto &[key, value] : map ) keys.push_back( key );
  std::sort( keys.begin(), keys.end() );
  return keys;
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

    // The mappings live in unordered maps, so walk them in id order: that lands the controls in
    // the order they appear on the pad, which reads better than alphabetical names. Sorting ids
    // rather than the built messages keeps the id out of the message.
    for ( const int index : sortedKeys( config.button_mappings ) ) {
      const auto &mapping = config.button_mappings.at( index );
      hector_gamepad_manager_msgs::msg::GamepadButtonMapping button_msg;
      button_msg.name = buttonName( index );
      button_msg.plugin = pluginId( mapping.plugin );
      addAction( button_msg.actions, GamepadAction::EVENT_PRESS, mapping.on_press );
      addAction( button_msg.actions, GamepadAction::EVENT_DOUBLE_PRESS, mapping.on_double_press );
      addAction( button_msg.actions, GamepadAction::EVENT_HOLD, mapping.on_hold );
      addAction( button_msg.actions, GamepadAction::EVENT_RELEASE, mapping.on_release );
      config_msg.buttons.push_back( button_msg );
    }

    for ( const int index : sortedKeys( config.axis_mappings ) ) {
      const auto &mapping = config.axis_mappings.at( index );
      hector_gamepad_manager_msgs::msg::GamepadAxisMapping axis_msg;
      axis_msg.name = axisName( index );
      axis_msg.plugin = pluginId( mapping.plugin );
      axis_msg.function = mapping.function_name;
      axis_msg.description = mapping.description;
      config_msg.axes.push_back( axis_msg );
    }

    msg.configs.push_back( config_msg );
  }

  for ( size_t i = 0; i < config_switches.size(); ++i ) {
    if ( config_switches[i].config.empty() )
      continue;
    hector_gamepad_manager_msgs::msg::GamepadConfigSwitch switch_msg;
    switch_msg.name = buttonName( static_cast<int>( i ) );
    switch_msg.config = config_switches[i].config;
    switch_msg.description = config_switches[i].description;
    msg.config_switches.push_back( switch_msg );
  }

  return msg;
}
} // namespace hector_gamepad_manager
