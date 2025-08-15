#ifndef HECTOR_GAMEPAD_MANAGER_GAMEPAD_FUNCTION_PLUGIN_HPP
#define HECTOR_GAMEPAD_MANAGER_GAMEPAD_FUNCTION_PLUGIN_HPP

#include "blackboard.hpp"
#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace hector_gamepad_plugin_interface
{
class GamepadFunctionPlugin
{
public:
  // Destructor
  virtual ~GamepadFunctionPlugin() = default;

  void initializePlugin( const rclcpp::Node::SharedPtr &node, const std::string &plugin_id,
                         std::shared_ptr<Blackboard> blackborad )
  {
    node_ = node;
    blackboard_ = blackborad;
    setPluginId( plugin_id );
    initialize( node );
  }

  /**
   * @brief Handle button input events.
   *
   * @param function The function name that is associated with the button.
   * @param pressed True if the button is pressed, false otherwise.
   */
  virtual void handleButton( const std::string &function, const std::string &id, const bool pressed )
  {
    const std::string unique_function = function + "_" + id;
    if ( button_states_.count( unique_function ) == 0 ) {
      button_states_[unique_function] = false;
    }

    if ( pressed ) {
      if ( button_states_[unique_function] ) {
        handleHold( function, id );
      } else {
        handlePress( function, id );
      }
    } else {
      if ( button_states_[unique_function] ) {
        handleRelease( function, id );
      }
    }

    button_states_[unique_function] = pressed;
  }

  /**
   * @brief Handle axis input events.
   *
   * @param function The function name that is associated with the axis.
   * @param value The value of the axis input event.
   */
  virtual void handleAxis( const std::string &function, const std::string &id, double value )
  {
    (void)function;
    (void)id;
    (void)value;
  }

  /**
   * @brief Function for handling input events once if the button or axis is pressed.
   *
   * @param function The name of the function that is associated with the input event.
   */
  virtual void handlePress( const std::string &function, const std::string &id )
  {
    (void)function;
    (void)id;
  }

  /**
   * @brief Function for handling input events where the button or axis is held down.
   *
   * @param function The name of the function that is associated with the input event.
   */
  virtual void handleHold( const std::string &function, const std::string &id )
  {
    (void)function;
    (void)id;
  }

  /**
   * @brief Function for handling input events where the button or axis is released.
   *
   * @param function The name of the function that is associated with the input event.
   */
  virtual void handleRelease( const std::string &function, const std::string &id )
  {
    (void)function;
    (void)id;
  }

  template<typename T>
  T getConfigValueOr( const std::string &id, const std::string &param,
                      const T &default_value = T() ) const
  {
    std::string key = plugin_id_ + "_" + id + "/" + param;
    return blackboard_->value_or<T>( key, default_value );
  }

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

  /**
   * @brief Get the plugin ID.
   * @return The plugin ID.
   */
  std::string getPluginId() const { return plugin_id_; }

  /**
   * @brief Get the plugin name.
   * @return The plugin name.
   */
  std::string getPluginName() const { return plugin_name_; }

  /**
   * @brief Get the plugin namespace.
   * @return The plugin namespace.
   */
  std::string getPluginNamespace() const { return plugin_namespace_; }

protected:
  /**
   * @brief Initialize function that is called when the plugin is loaded.
   *
   * @param node The ROS node that the plugin is associated with.
   */
  virtual void initialize( const rclcpp::Node::SharedPtr &node ) = 0;
  void setPluginId( const std::string &plugin_id )
  {
    plugin_id_ = plugin_id;
    // extract name and namespace from plugin_id
    // e.g. "hector_gamepad_manager_plugins::moveit_plugin" -> name: "moveit_plugin", namespace: "hector_gamepad_manager_plugins"
    size_t pos = plugin_id_.find( "::" );
    if ( pos != std::string::npos ) {
      plugin_name_ = plugin_id_.substr( pos + 2 );
      plugin_namespace_ = plugin_id_.substr( 0, pos );
    } else {
      plugin_name_ = plugin_id_;
      plugin_namespace_ = "";
    }
    // make sure the plugin name is snake_case
    plugin_name_ = camel_to_snake( plugin_name_ );
  }
  std::string camel_to_snake( const std::string &input )
  {
    std::string result;
    result.reserve( input.size() * 2 ); // conservative allocation
    for ( size_t i = 0; i < input.size(); ++i ) {
      char c = input[i];
      if ( std::isupper( static_cast<unsigned char>( c ) ) ) {
        if ( i > 0 ) {
          result.push_back( '_' );
        }
        result.push_back( static_cast<char>( std::tolower( static_cast<unsigned char>( c ) ) ) );
      } else {
        result.push_back( c );
      }
    }
    return result;
  }
  // The ROS node
  rclcpp::Node::SharedPtr node_;

  // Specifies if the plugin is active.
  bool active_ = false;

  // The current state of the buttons per function.
  std::unordered_map<std::string, bool> button_states_;
  std::string plugin_id_;
  std::string plugin_name_;
  std::string plugin_namespace_;
  std::shared_ptr<Blackboard> blackboard_;
};
} // namespace hector_gamepad_plugin_interface

#endif // HECTOR_GAMEPAD_MANAGER_GAMEPAD_FUNCTION_PLUGIN_HPP
