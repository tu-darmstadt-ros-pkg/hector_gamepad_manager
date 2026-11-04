#ifndef HECTOR_GAMEPAD_MANAGER_PLUGINS_BLACKBOARD_KEYS_HPP
#define HECTOR_GAMEPAD_MANAGER_PLUGINS_BLACKBOARD_KEYS_HPP

#include <hector_gamepad_plugin_interface/blackboard.hpp>

#include <string>

namespace hector_gamepad_manager_plugins
{
// std::string: the MoveIt group the MoveitPlugin is currently moving to a named pose (from the
// button press until its release), absent otherwise. Teleop plugins of the joints of that group
// ignore their input meanwhile, since their controller switch would abort the MoveIt trajectory.
constexpr char kMoveitActiveGroupKey[] = "moveit_active_group";

// Whether the MoveitPlugin is currently moving `group`, see kMoveitActiveGroupKey.
inline bool isMovedByMoveit( const hector_gamepad_plugin_interface::Blackboard &blackboard,
                             const std::string &group )
{
  const auto *active_group = blackboard.try_get<std::string>( kMoveitActiveGroupKey );
  return active_group != nullptr && *active_group == group;
}
} // namespace hector_gamepad_manager_plugins

#endif // HECTOR_GAMEPAD_MANAGER_PLUGINS_BLACKBOARD_KEYS_HPP
