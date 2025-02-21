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

## Plugins
See the hector_gamepad_manager_plugins pkg for available plugins.