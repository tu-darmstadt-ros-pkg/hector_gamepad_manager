#ifndef HECTOR_GAMEPAD_MANAGER_GAMEPAD_CONFIG_HPP
#define HECTOR_GAMEPAD_MANAGER_GAMEPAD_CONFIG_HPP

#include "hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>

namespace hector_gamepad_manager
{
using GamepadFunctionPlugin = hector_gamepad_plugin_interface::GamepadFunctionPlugin;

// Total addressable buttons: indices 0-10 physical, 11-24 virtual (axis deflection past a deadzone).
constexpr std::size_t kNumButtons = 25;

// A single plugin function bound to one button event, with its human-readable description.
struct ActionMapping {
  std::string function;
  std::string description;

  bool empty() const { return function.empty(); }
};

// Mapping of an axis to a function of a plugin.
struct FunctionMapping {
  std::shared_ptr<GamepadFunctionPlugin> plugin;
  std::string function_name;
  std::string description;
};

// For double-press buttons the manager bypasses handleButton and dispatches handlePress /
// handleHold / handleRelease directly, so plugins must not rely on button_states_ for them.
struct ButtonFunctionMapping {
  std::shared_ptr<GamepadFunctionPlugin> plugin;

  ActionMapping on_press;        // called on initial press
  ActionMapping on_double_press; // called on double press (empty = disabled)
  ActionMapping on_hold;         // called while held (empty = uses on_press)
  ActionMapping on_release;      // called on release (empty = uses on_press)

  bool has_double_press() const { return !on_double_press.empty(); }
};

// A button that switches the active configuration.
struct ConfigSwitch {
  std::string config;
  std::string description;
};

struct GamepadConfig {
  std::string description; // optional top-level description of the config, from YAML; may be empty
  std::unordered_map<int, ButtonFunctionMapping> button_mappings;
  std::unordered_map<int, FunctionMapping> axis_mappings;
};
} // namespace hector_gamepad_manager

#endif // HECTOR_GAMEPAD_MANAGER_GAMEPAD_CONFIG_HPP
