# hector_gamepad_plugin_interface

Provides the plugin base class (`hector_gamepad_plugin_interface::GamepadFunctionPlugin`) that all
`hector_gamepad_manager` plugins derive from, plus the shared helpers they use: the `Blackboard`,
the `FeedbackManager` (rumble), and access to the `controller_orchestrator`.

Plugins are loaded by the manager via `pluginlib` and react to gamepad input by implementing the
handler methods below. The manager runs on the robot, in the robot namespace, so any ROS endpoints a
plugin creates on `node_` should use **relative** names - they resolve under the robot namespace.

## Writing a plugin

Derive from `GamepadFunctionPlugin`, export it with `pluginlib`, and implement the relevant methods:

```cpp
class MyPlugin final : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{
  void initialize( const rclcpp::Node::SharedPtr &node ) override;  // set up publishers/clients
  void handlePress( const std::string &function, const std::string &id ) override;
  void handleHold( const std::string &function, const std::string &id ) override;
  void handleRelease( const std::string &function, const std::string &id ) override;
  void handleAxis( const std::string &function, const std::string &id, double value ) override;
  void update() override;        // called periodically after inputs are dispatched
  void activate() override;      // unlock the plugin
  void deactivate() override;    // lock the plugin and bring it into a safe state
};
```

`function` is the function string from the config (e.g. `drive`, `go_to_pose`); `id` is the config id
that uniquely scopes the mapping so the same function can be reused multiple times. `update`,
`activate` and `deactivate` are required; the input handlers are optional (default to no-op).

### Lifecycle

The manager calls `initializePlugin(...)` once when the plugin is loaded. This stores the shared
resources, sets the plugin id/name/namespace from the registered class name, and then calls the
plugin's `initialize(node)`. Plugins belonging to the active configuration are `activate()`d; plugins
of other configurations are `deactivate()`d when switching configs.

## Provided to the plugin

Available as protected members after initialization:

| Member | Description |
| --- | --- |
| `node_` | ROS node in the robot namespace. Create publishers/subscriptions/clients here with relative names. |
| `blackboard_` | Shared key/value store for exchanging state between plugins (e.g. `inverted_steering`). |
| `feedback_manager_` | Triggers gamepad rumble via named vibration patterns. |
| `controller_orchestrator_` | Activates/queries ros2_control controllers (see helpers below). |
| `plugin_id_` / `plugin_name_` / `plugin_namespace_` | The registered class name and its split parts (name is converted to `snake_case`). |
| `active_` | Whether the plugin is currently active. |
| `button_states_` | Per-function button state used by the default `handleButton` dispatch. |

> **Note:** For buttons configured with `on_double_press`, the manager dispatches
> `handlePress`/`handleHold`/`handleRelease` directly and bypasses `handleButton`, so `button_states_`
> does not reflect the physical state of those buttons. See the manager README for details.

## Helper methods

- `getConfigValueOr<T>(id, param, default)` - read an `args` value for this plugin/config from the
  blackboard, namespaced by plugin id and config id.
- `activateControllers(names, callback = nullptr)` - switch ros2_control controllers asynchronously.
- `areControllersActive(names)` - check whether the given controllers are active.
- `isActive()`, `getPluginId()`, `getPluginName()`, `getPluginNamespace()` - accessors.
