#ifndef HECTOR_GAMEPAD_MANAGER_GAMEPAD_MAPPING_BUILDER_HPP
#define HECTOR_GAMEPAD_MANAGER_GAMEPAD_MAPPING_BUILDER_HPP

#include "hector_gamepad_manager/gamepad_config.hpp"

#include <hector_gamepad_manager_msgs/msg/gamepad_mapping.hpp>

#include <array>
#include <map>
#include <string>

namespace hector_gamepad_manager
{
// Build the static GamepadMapping message from the loaded configs.
// Kept free of any ROS node so the message content can be unit-tested directly.
hector_gamepad_manager_msgs::msg::GamepadMapping
buildGamepadMappingMsg( const std::map<std::string, GamepadConfig> &configs,
                        const std::array<ConfigSwitch, kNumButtons> &config_switches,
                        const std::string &default_config );
} // namespace hector_gamepad_manager

#endif // HECTOR_GAMEPAD_MANAGER_GAMEPAD_MAPPING_BUILDER_HPP
