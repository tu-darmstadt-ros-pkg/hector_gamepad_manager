#include "hector_gamepad_manager/joy_satellite.hpp"

#include <rclcpp_components/register_node_macro.hpp>

// Exposes the same node as the standalone joy_satellite_node executable as a
// composable component, so it can be loaded into a shared rclcpp_components
// container (intra-process, single process with other operator-station nodes).
RCLCPP_COMPONENTS_REGISTER_NODE( hector_gamepad_manager::JoySatelliteNode )
