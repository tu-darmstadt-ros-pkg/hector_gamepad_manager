#ifndef HECTOR_GAMEPAD_MANAGER_GAMEPAD_FUNCTION_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_GAMEPAD_FUNCTION_PLUGIN_HPP

#include <rclcpp/rclcpp.hpp>

namespace hector_gamepad_manager
{
class GamepadFunctionPlugin
{
public:
  // Destructor
  virtual ~GamepadFunctionPlugin() = default;

  /**
   * @brief Initialize function that is called when the plugin is loaded.
   *
   * @param node The ROS node that the plugin is associated with.
   * @param active True if the plugin should be active on startup, false otherwise.
   */
  virtual void initialize( const rclcpp::Node::SharedPtr &node, const bool active ) = 0;

  /**
   * @brief Handle button input events.
   *
   * @param function The function name that is associated with the button.
   * @param pressed True if the button is pressed, false otherwise.
   */
  virtual void handleButton( const std::string &function, const bool pressed ) = 0;

  /**
   * @brief Handle axis input events.
   *
   * @param function The function name that is associated with the axis.
   * @param value The value of the axis input event.
   */
  virtual void handleAxis( const std::string &function, const double value ) = 0;

  /**
   * @brief Update function that is called periodically to update the plugin state after handling input events.
   */
  virtual void update() = 0;

  /**
   * @brief Activate function to unlock the plugin.
   */
  virtual void activate() = 0;

  /**
   * @brief Deactivate function to lock the plugin and bring it into a safe state.
   */
  virtual void deactivate() = 0;

protected:
  // Specifies if the plugin is active.
  bool active_;
};
} // namespace hector_gamepad_manager

#endif // HECTOR_GAMEPAD_MANAGER_GAMEPAD_FUNCTION_PLUGIN_HPP
