#ifndef HECTOR_GAMEPAD_MANAGER_GAMEPAD_FUNCTION_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_GAMEPAD_FUNCTION_PLUGIN_HPP

#include <rclcpp/rclcpp.hpp>

namespace hector_gamepad_manager_interface
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
   */
  virtual void initialize( const rclcpp::Node::SharedPtr &node ) = 0;

  /**
   * @brief Returns name of associated plugin
   *
   */
  virtual std::string getPluginName()=0;

  /**
   * @brief Handle button input events.
   *
   * @param function The function name that is associated with the button.
   * @param pressed True if the button is pressed, false otherwise.
   */
  virtual void handleButton( const std::string &function, const bool pressed )
  {
    if ( button_states_.count( function ) == 0 ) {
      button_states_[function] = false;
    }

    if ( pressed ) {
      if ( button_states_[function] ) {
        handleHold( function );
      } else {
        handlePress( function );
      }
    } else {
      if ( button_states_[function] ) {
        handleRelease( function );
      }
    }

    button_states_[function] = pressed;
  }

  /**
   * @brief Handle axis input events.
   *
   * @param function The function name that is associated with the axis.
   * @param value The value of the axis input event.
   */
  virtual void handleAxis( const std::string &function, double value )
  {
    (void)function;
    (void)value;
  }

  /**
   * @brief Function for handling input events once if the button or axis is pressed.
   *
   * @param function The name of the function that is associated with the input event.
   */
  virtual void handlePress( const std::string &function ) { (void)function; }

  /**
   * @brief Function for handling input events where the button or axis is held down.
   *
   * @param function The name of the function that is associated with the input event.
   */
  virtual void handleHold( const std::string &function ) { (void)function; }

  /**
   * @brief Function for handling input events where the button or axis is released.
   *
   * @param function The name of the function that is associated with the input event.
   */
  virtual void handleRelease( const std::string &function ) { (void)function; }

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

  /**
   * @brief Check if the plugin is active.
   * @return True if the plugin is active, false otherwise.
   */
  bool isActive() const { return active_; }

protected:
  // The ROS node
  rclcpp::Node::SharedPtr node_;

  // Specifies if the plugin is active.
  bool active_ = false;

  // The current state of the buttons per function.
  std::unordered_map<std::string, bool> button_states_;
};
} // namespace hector_gamepad_manager_interface

#endif // HECTOR_GAMEPAD_MANAGER_GAMEPAD_FUNCTION_PLUGIN_HPP
