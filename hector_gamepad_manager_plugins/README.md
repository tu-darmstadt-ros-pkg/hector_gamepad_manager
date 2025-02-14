# hector_gamepad_manager_plugins

This package contains the following plugins for the hector_gamepad_manager package.
The functionalities per plugin are described below.

## DrivePlugin

This plugin is used to drive the robot using a gamepad.

### Functions

- `drive`: Forward and backward movement of the robot.
- `steer`: Left and right steering of the robot.
- `fast`: Fast driving mode
- `slow`: Slow driving mode

## Manipulation Plugin

This plugin can be used to control the robots end-effector and gripper.
Additionally, in the hold mode the robot can be driven while the end-effector remains at its hold position.

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
- `hold_mode`: Toggle the hold mode. In hold mode the robot can be driven while the end-effector remains at its hold position.