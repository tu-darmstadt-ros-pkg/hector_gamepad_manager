#ifndef HECTOR_GAMEPAD_MANAGER_GAMEPAD_BUTTONS_HPP
#define HECTOR_GAMEPAD_MANAGER_GAMEPAD_BUTTONS_HPP

#include "hector_gamepad_manager/gamepad_config.hpp"

#include <map>
#include <string>
#include <vector>

namespace hector_gamepad_manager
{
// Canonical names for the gamepad's inputs - the identifier shared by the config files, the
// published GamepadMapping message and whatever renders it. Ids stay inside the Joy adapter and
// follow SDL's GameController layout, which `game_controller_node` publishes (never `joy_node`).

//! "axis_buttons" YAML key -> internal button id (kVirtualButtonBase + offset). The offsets match
//! the assignment order in HectorGamepadManager::convertJoyToGamepadInputs().
const std::map<std::string, int> &axisButtonIds();

//! Canonical name of a button id, or "" if the id has no canonical meaning.
std::string buttonName( int id );

//! Canonical name of an axis id, or "" if the id is out of range.
std::string axisName( int id );

//! Button id for a canonical name, physical or virtual, or -1 if the name is unknown.
int buttonId( const std::string &name );

//! Axis id for a canonical name, or -1 if the name is unknown.
int axisId( const std::string &name );

//! True for the two trigger axes, which are the only axes the Joy adapter has to convert.
//!
//! A trigger reads 0 (released) to 1 (fully pressed); the sticks read -1 to 1. On the wire a
//! pressed trigger arrives as -1 instead: SDL reports triggers as 0..32767 and never negative,
//! and game_controller_node scales every axis by one negative factor. convertJoyToGamepadInputs()
//! flips them back, so nothing downstream sees the wire sign.
bool isTriggerAxis( int id );

//! Every valid button name, physical then virtual, for error messages.
std::vector<std::string> buttonNames();

//! Every valid axis name, for error messages.
std::vector<std::string> axisNames();

// Identity of one binding: it namespaces the binding's `args` on the blackboard at load time and
// tells the plugin which binding fired at dispatch time. Those two sites are far apart, so both
// go through these functions - a mismatch would store args under a key nothing ever reads.
//
// Button and axis ids are kept apart because the two share a name space: "left_trigger" is both
// an axis and the virtual button derived from it, and a config may bind them differently.

//! Blackboard/dispatch id of a button binding in `config_name`.
std::string buttonBindingId( const std::string &config_name, const std::string &button_name );

//! Blackboard/dispatch id of an axis binding in `config_name`.
std::string axisBindingId( const std::string &config_name, const std::string &axis_name );
} // namespace hector_gamepad_manager

#endif // HECTOR_GAMEPAD_MANAGER_GAMEPAD_BUTTONS_HPP
