#ifndef HECTOR_GAMEPAD_MANAGER_GAMEPAD_CONFIG_HPP
#define HECTOR_GAMEPAD_MANAGER_GAMEPAD_CONFIG_HPP

#include "hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp"

#include <cstddef>
#include <map>
#include <memory>
#include <string>

namespace hector_gamepad_manager
{
using GamepadFunctionPlugin = hector_gamepad_plugin_interface::GamepadFunctionPlugin;

// Physical buttons occupy ids [0, kVirtualButtonBase) and are read 1:1 from the Joy message.
// Axis-derived virtual buttons (a stick or trigger deflected past a deadzone) start at
// kVirtualButtonBase, so extending the physical range never shifts them. Both are keyed by name
// in the config files - see gamepad_buttons.hpp for the catalog and the expected Joy source.
constexpr std::size_t kVirtualButtonBase = 32;
constexpr std::size_t kNumVirtualButtons = 10;
constexpr std::size_t kNumButtons = kVirtualButtonBase + kNumVirtualButtons;
constexpr std::size_t kNumAxes = 6;

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
  // Identity of this binding, from axisBindingId(). Built once when the config is read so the
  // dispatch site cannot derive a different one than the site that stored the binding's args.
  std::string binding_id;
};

// The manager owns the edge detection and dispatches handlePress / handleHold / handleRelease, so
// a plugin sees the events and never a raw button level to derive them from.
struct ButtonFunctionMapping {
  std::shared_ptr<GamepadFunctionPlugin> plugin;

  ActionMapping on_press;        // called on initial press
  ActionMapping on_double_press; // called on double press (empty = disabled)
  ActionMapping on_hold;         // called while held (empty = uses on_press)
  ActionMapping on_release;      // called on release (empty = uses on_press)

  // As FunctionMapping::binding_id, from buttonBindingId().
  std::string binding_id;

  bool has_double_press() const { return !on_double_press.empty(); }

  // The function an event dispatches to, with the fallback to on_press applied. Kept as accessors
  // rather than filled in at load time because an unset on_hold/on_release also means "do not
  // advertise this event in the published mapping" - see addAction() in gamepad_mapping_builder.
  const std::string &holdFunction() const
  {
    return on_hold.empty() ? on_press.function : on_hold.function;
  }

  const std::string &releaseFunction() const
  {
    return on_release.empty() ? on_press.function : on_release.function;
  }
};

// A button that switches the active configuration.
struct ConfigSwitch {
  std::string config;
  std::string description;
};

struct GamepadConfig {
  std::string description; // optional top-level description of the config, from YAML; may be empty
  // Ordered by button/axis id, which is the order the controls sit on the pad: iterating a config
  // - to dispatch it, or to publish it - needs no sorting step and reads the same every run.
  std::map<int, ButtonFunctionMapping> button_mappings;
  std::map<int, FunctionMapping> axis_mappings;
};
} // namespace hector_gamepad_manager

#endif // HECTOR_GAMEPAD_MANAGER_GAMEPAD_CONFIG_HPP
