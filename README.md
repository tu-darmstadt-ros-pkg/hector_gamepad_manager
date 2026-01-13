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

#### Example: Driving Configuration

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
- `fast`: Fast driving mode. Default button: (A)
- `slow`: Slow driving mode. Default button: (X)

## FlipperPlugin

This plugin is used to steer the robot flippers using a gamepad.

### Functions

- `front_flippers_up`: Rotate front flippers upwards (counterclockwise). Default button: (RB)
- `front_flippers_down`: Rotate front flippers downwards (clockwise). Default button: (LT)
- `back_flippers_up`: Rotate back flippers upwards (clockwise). Default button: (LB)
- `back_flippers_down`: Rotate back flippers downwards (counterclockwise). Default button: (LT)

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
