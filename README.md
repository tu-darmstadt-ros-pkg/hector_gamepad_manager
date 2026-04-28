# Hector Gamepad Manager

The `hector_gamepad_manager` package receives `sensor_msgs::Joy` messages and dispatches them to **plugins** that react
to gamepad input.
For example, the `DrivePlugin` reacts to joystick movements and sends `cmd_vel` commands to the robot.

---

## Configuration

The system supports multiple **configurations** for different capabilities, such as **driving**, **manipulation**, or *
*flipper control**.
A **meta-configuration file** maps gamepad buttons to these configurations.

When a mapped button is pressed, the manager switches to the corresponding configuration.
⚠️ Buttons used for switching configurations are ignored in the individual configs themselves.

### Example: Meta Configuration File

```yaml
buttons:
  6: # Button Back
    package: "hector_gamepad_manager"
    config: "driving"

  7: # Button Start
    package: "hector_gamepad_manager"
    config: "manipulation"
```

---

### Individual Configuration Files

Each configuration file maps **buttons/axes** to **plugin functions**.
Functions can now include **parameters** via the `args` field. These parameters are stored in the shared **blackboard**
and are uniquely namespaced, so you can reuse the same function in different or even within the same config.

#### Example: Driving Configuration (basic format)

```yaml
axes:
  0: # Left joystick left/right
    plugin: "hector_gamepad_manager_plugins::DrivePlugin"
    function: "steer"

  1: # Left joystick up/down
    plugin: "hector_gamepad_manager_plugins::DrivePlugin"
    function: "drive"

buttons:
  0: # Button A
    plugin: "hector_gamepad_manager_plugins::DrivePlugin"
    function: "fast"

  1: # Button B
    plugin: "hector_gamepad_manager_plugins::MoveitPlugin"
    function: "go_to_pose"
    args:
      group: "arm_group"
      pose: "folded"
```

#### Per-Event Button Mapping

Buttons support an extended format that maps different functions to different input events.
This enables features like **double-press detection** without requiring plugin-side changes.

```yaml
buttons:
  4: # Button LB
    plugin: "hector_gamepad_manager_plugins::FlipperPlugin"
    on_press:
      function: "flipper_back_up"
    on_double_press:
      function: "flipper_back_upright"
    on_hold:                          # optional, defaults to on_press function
      function: "flipper_back_up"
    on_release:                       # optional, defaults to on_press function
      function: "flipper_back_up"
    args:                             # shared by all events on this button
      some_param: 1.0
```

Available event types:
- **`on_press`** *(required)*: Triggered on button press (single press). This is equivalent to the basic `function` field. It is also the fallback target for `on_hold`/`on_release` and the dispatch target on a single tap when `on_double_press` is configured, so a new-format mapping without `on_press` is rejected at load time.
- **`on_double_press`**: Triggered when the button is pressed twice within a short time window. The window length is configurable via the `double_press_window_sec` ROS parameter (default 0.25s). When configured, `on_press` is delayed until the window expires to avoid false triggers.
- **`on_hold`**: Triggered repeatedly while the button is held down. Defaults to the `on_press` function if omitted.
- **`on_release`**: Triggered when the button is released. Defaults to the `on_press` function if omitted.

Args are placed at the top level of the button entry (not under individual events) and are shared by all events on that button. As a convenience, an `args` block under `on_press` is also accepted and treated identically. Per-event `args` under `on_double_press`, `on_hold`, or `on_release` are not currently distinguishable on the plugin read side and will be ignored with a warning.

> **Note:** Buttons without `on_double_press` are dispatched immediately with zero added latency.
> The basic format (`plugin` + `function`) is still fully supported and behaves identically to before.

#### Caveats

- **Latency on double-press buttons.** Configuring `on_double_press` delays single-press dispatch by up to `double_press_window_sec` (default 0.25s) so the manager can wait for a possible second press. Avoid `on_double_press` on time-critical actions (e.g. emergency stops).
- **Per-event args are not supported.** A button has one `args:` block shared by all events. `args:` placed under `on_double_press`, `on_hold`, or `on_release` are ignored with a warning at load time.
- **`double_press_window_sec` is read once at startup**, not dynamically reconfigurable.
- **Plugin state contract.** For buttons with `on_double_press`, the manager dispatches `handlePress`/`handleHold`/`handleRelease` directly and bypasses the base class's `handleButton`. Plugin subclasses must not assume `GamepadFunctionPlugin::button_states_` reflects the physical state of double-press-configured buttons.

---

## Joy Callback

When a `sensor_msgs::Joy` message arrives, all plugins in the active configuration are invoked with the corresponding
button/axis values.
Plugins can then react to input and send commands to the robot.

## Launch

The hector_gamepad_manager can be launched with the following command:

```bash
ros2 launch hector_gampepad_manager hector_gamepad_manager.launch.yaml
```

## Blackboard

Plugins can use the blackboard to exchange data with each other. The blackboard is a shared data structure that allows
plugins to store and retrieve data. This is useful for sharing state or configuration between plugins without direct
dependencies. For example, the DrivePlugin can store an inverted steering state in the blackboard, which can
then be accessed by other plugins. For instance, the FlipperPlugin can check if the steering is inverted and switch the
commands to the front and back flippers.

---

# Hector Gamepad Manager Plugins

This package contains the following plugins for the hector_gamepad_manager package.
The functionalities per plugin are described below.

## DrivePlugin

This plugin is used to drive the robot using a gamepad.

### Functions

- `drive`: Forward and backward movement of the robot. Default button: (Left joystick up/down)
- `steer`: Left and right steering of the robot. Default button: (Left joystick left/right)
- `fast`: Fast driving mode. Default button: (Y)
- `slow`: Slow driving mode. Default button: (A)

> **Default-mapping change:** `fast` and `slow` were swapped from (A/X) to (Y/A) when the per-event button format landed, freeing up B and X for the FlipperPlugin's individual-control modes and double-press sync actions.

## FlipperPlugin

This plugin is used to steer the robot flippers using a gamepad. It sends velocity commands for manual control and uses action clients to trigger drive-to-upright and sync operations on the `flipper_velocity_to_position_controller`.

### Functions

#### Velocity Control
- `flipper_front_up`: Rotate front flippers upwards (counterclockwise). Default button: (RB)
- `flipper_front_down`: Rotate front flippers downwards (clockwise). Default button: (RT)
- `flipper_back_up`: Rotate back flippers upwards (clockwise). Default button: (LB)
- `flipper_back_down`: Rotate back flippers downwards (counterclockwise). Default button: (LT)

#### Drive to Upright (via `DriveFlipperGroup` action)
- `flipper_front_upright`: Drive front flippers to upright position. Default trigger: double-press (RB)
- `flipper_back_upright`: Drive back flippers to upright position. Default trigger: double-press (LB)

#### Sync Flippers (via `SyncFlipperGroup` action)
- `sync_front_flippers`: Sync front flipper pair to their average position. Default trigger: double-press (B)
- `sync_back_flippers`: Sync back flipper pair to their average position. Default trigger: double-press (X)
- `sync_all_flippers`: Sync all flipper groups simultaneously. (Not mapped by default)

Running drive-to-upright and sync actions are **automatically pre-empted on the controller side** as soon as a manual velocity command arrives for the affected flipper group (e.g., pressing LB/RB or moving LT/RT). The plugin itself does not cancel goals; the `flipper_velocity_to_position_controller` aborts the active action when it sees an incoming velocity command.

#### Individual Control Modes
- `individual_front_flipper_control_mode`: Hold to enable individual front flipper steering; release to return to paired control. Default button: (B, single-press)
- `individual_back_flipper_control_mode`: Hold to enable individual back flipper steering; release to return to paired control. Default button: (X, single-press)

> **Note on B/X and LB/RB:** these buttons each serve two functions via the per-event format. Holding B/X enables individual front/back control mode until released; pressing and holding LB/RB drives a velocity command. A double press triggers the corresponding sync or drive-to-upright action. Because `on_double_press` is configured on these buttons, single-press dispatch is delayed by `double_press_window_sec` (default 0.25s) — a quick tap therefore enables the mode (or velocity command) only briefly before its paired release fires.

### Parameters

- `command_topic` (string): Topic to publish velocity commands. Default: `flipper_velocity_controller/commands`
- `drive_flipper_action` (string): Action server for drive-to-target. Default: `/<robot_ns>/flipper_velocity_to_position_controller/drive_flipper_group`
- `sync_flipper_action` (string): Action server for sync. Default: `/<robot_ns>/flipper_velocity_to_position_controller/sync_flipper_group`

## Manipulation Plugin

This plugin can be used to control the robots end-effector and gripper.
Additionally, in the hold mode the robot can be driven while the end-effector remains at its current position in the
world frame.

### Functions

#### Axis

- `move_left_right`: Move the end-effector left and right. Or move the robot base while the hold button is pressed.
- `move_up_down`: Move the end-effector up and down. Or move the robot base while the hold button is pressed.
- `move_forward`: Move the end-effector forward.
- `move_backward`: Move the end-effector backward.
- `rotate_pitch`: Rotate the end-effector around the pitch axis.
- `rotate_yaw`: Rotate the end-effector around the yaw axis.

#### Buttons

- `rotate_roll_clockwise`: Rotate the end-effector clockwise around the roll axis.
- `rotate_roll_counterclockwise`: Rotate the end-effector counterclockwise around the roll axis.
- `open_gripper`: Open the gripper.
- `close_gripper`: Close the gripper.
- `hold_mode`: Toggle the hold mode. In hold mode the robot can be driven while the end-effector remains at its current
  position in the world frame.

## BlackboardPlugin

This plugin allows assigning buttons to directly manipulate variables stored in the shared **blackboard**.
It supports toggling and holding boolean values as well as setting string values.

### Functions

#### Buttons

* `toggle`: Toggles a boolean variable in the blackboard.
    - `args`:
        - `name`: The name of the variable to toggle.
        - `initial`: Optional, the initial value of the variable if it does not exist yet (default is `false`).
        - `topic`: Optional, if set the plugin will publish the new value to the specified topic whenever it changes (
          transient_local, std_msgs/Bool).
    - Example: `toggle` with name inverted_steering → toggles the `inverted_steering` variable between `true` and
      `false`.
* `hold`: Sets a boolean variable in the blackboard to `true` while the button is pressed, and resets it to `false`
  when released.
    - `args`:
        - `name`: The name of the variable to hold.
    - Example: `hold` with name safety_mode → sets `safety_mode = true` when held, and resets it to `false` when
      released.
* `set`: Sets a string variable in the blackboard to a specific value when the button is pressed.
    - `args`:
        - `name`: The name of the variable to set.
        - `to`: The value to set the variable to.
          Example:
       ```bash
      plugin: "hector_gamepad_manager_plugins::BlackboardPlugin"
      function: "set"
      args:
        name: "robot_mode"
        to: "manipulation"
       ```
      Sets the `robot_mode` variable in the blackboard to `manipulation` when the button is pressed.

## MoveItPlugin

Executes MoveIt! commands to position the robot in predefined SRDF poses.

### Functions

### Buttons

- `go_to_pose`: Moves the robot to a predefined pose.
    - `args`:
        - `group`: The MoveIt! group to control (e.g., "arm_group").
        - `pose`: The name of the pose to move to (e.g., "folded").
        - `inverted_pose`: Optional, if set the robot will move to the inverted pose if `inverted_steering` from the
          blackboard is `true`.

## BatteryMonitorPlugin

Monitors cell voltages (any battery message type via `ros_babel_fish`) and vibrates the gamepad when any cell
falls below a configured threshold. The vibration can be muted temporarily via a button.

### Functions

- `mute` (button): Mutes vibration feedback for the configured duration.

### Parameters

- `battery_topic` (string): Topic to subscribe to. Default: `battery_status`.
- `cell_voltage_fields` (string list): Field paths (e.g., `["cell_voltages_battery1_mv", "cell_voltages_battery2_mv"]`)
  containing cell voltage values or arrays to evaluate.
- `low_cell_threshold` (double): Cell voltage threshold to trigger vibration. Default: `3500.0`.
- `vibration_pattern.on_durations_sec` (double array): Pulse durations in seconds. Default: `[0.2, 0.2]`.
- `vibration_pattern.off_durations_sec` (double array): Break durations after each pulse in seconds. Default: `[0.2, 5.0]`.
- `vibration_pattern.intensity` (double): Rumble intensity in `[0.0, 1.0]`. Default: `0.8`.
- `vibration_pattern.cycle` (bool): If `true`, repeats the pattern forever. Default: `true`.
- `mute_duration_sec` (double): How long the mute lasts after pressing `mute`. Default: `300.0` seconds (5 minutes).
- `ignore_zero_voltage` (bool): If `true`, zero voltage readings are ignored (useful for unpopulated cells). Default: `true`.
- `ignore_nan_voltage` (bool): If `true`, NaN voltage readings are ignored. Default: `true`.

## Virtual Camera Plugin

Allows to pan and tilt a virtual camera which uses a geometry_msgs/msg/Transform similar as done in [image_projection](https://github.com/tu-darmstadt-ros-pkg/image_projection). Axis can pan and tilt, while a button hold can recenter the camera.
The plugin supports a front and back camera which are toggled with the `inverted_steering` mode.

### Functions

### Axis

- `move_left_right`: Pan the virtual camera.
- `move_up_down`: Tilt the virtual camera.

### Buttons
- `hold`: Recenter current camera

### Parameters

- `back_camera_topic_name` (string): ROS topic name for the back camera transform. Default: `virtual_camera/back`
- `front_camera_topic_name`(string): ROS topic name for the front camera transform. Default: `virtual_camera/front`
- `pan_speed` (double): Maximum pan speed in rad/s. Default: 0.5
- `tilt_speed` (double): Maximum tilt speed in rad/s. Default: 0.5
- `max_pan` (double): Maximum pan angle in rad. Default: 3.14
- `min_pan` (double): Minimum pan angle in rad. Default: -3.14
- `max_tilt` (double): Maximum tilt angle in rad. Default: 1.57
- `min_tilt` (double): Minimum tilt angle in rad. Default: -1.57
- `stick_hold_time` (double): Time in seconds to hold the stick to trigger recentering. Default: 1.0
- `invert_y_axis` (bool): Whether to invert the Y axis of the camera control. Default Y direction is downwards. Default: true
