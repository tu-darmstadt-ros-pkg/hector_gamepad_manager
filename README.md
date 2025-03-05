# Hector Game Pad Manager

This pkg receives sensor_msgs::Joy messages. Plugins can be loaded that react to this messages. For example the drive
message waits for joystick movements and sends cmd_vel commands to the robot. 

## Configuration
Several Configs can be defined that map button and axis buttons to the corresponding plugin functionalities. The active configuration can be dynamicylly changed by holding the CONFIG_SWITCH_BUTTON and pressing another button.
The Mapping between buttons and config files is defined in yaml file of the following format.
```yaml
buttons:
  0: # Button A
    config: "driving"

  1: # Button B
    config: "manipulation"
...
```
Each button maps to one config file that must be placed in the config folder of the hector_gamepad_manager pkg. The config file must be of the following format.
```yaml
axes:
  0: # Left joystick left/right
    plugin: "hector_gamepad_manager_plugins::DrivePlugin"
    function: "steer"

  1: # Left joystick up/down
    plugin: "hector_gamepad_manager_plugins::DrivePlugin"
    function: "drive"
...
buttons:
  0: # Button A
    plugin: "hector_gamepad_manager_plugins::DrivePlugin"
    function: "fast"

  1: # Button B
    plugin: ""
    function: ""
...
```
All config files are loaded on initialization. Every plugin mentioned in the config files is loaded and initialized.
>NOTE: The CONFIG_SWITCH_BUTTON will be ignored in all config files. If it is pressed no config will receive updates. The active plugins are disabled on configuration switches.

## JoyCallback
When a Joy message is received all plugins in the active config file are called with the corresponding axis/button values. The plugins can then react to the input and send commands to the robot.

## Launch
The hector_gamepad_manager can be launched with the following command:
```bash
ros2 launch hector_gampepad_manager hector_gamepad_manager.launch.yaml
```

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
Additionally, in the hold mode the robot can be driven while the end-effector remains at its current position in the world frame.

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
- `hold_mode`: Toggle the hold mode. In hold mode the robot can be driven while the end-effector remains at its current position in the world frame.