# Fake Joy Publisher

The **Fake Joy Publisher** is a ROS 2 utility node that emulates a physical gamepad for use with the [`hector_gamepad_manager`](https://github.com/TODO).
It publishes synthetic [`sensor_msgs/msg/Joy`](https://docs.ros2.org/latest/api/sensor_msgs/msg/Joy.html) messages on the same topic a real gamepad would, allowing automated tests, scripted input sequences, or teleoperation without real hardware.

---

## Features

* **Automatic integration with `hector_gamepad_manager`**

    * Finds the manager node by name (even with random suffixes in launch).
    * Retrieves `config_name` and `ocs_namespace` parameters automatically.
    * Loads meta-config and per-mode config YAML files.

* **Mode awareness & switching**

    * Understands multiple gamepad configurations ("modes").
    * Detects when an input belongs to a different mode and issues the correct **meta button press** to request a mode change.
    * Waits for the `active_config` confirmation from the manager before sending the intended inputs.

* **Axis and button remapping**

    * Handles special LT/RT trigger axes (2 & 5) that report `1.0` neutral → `-1.0` fully pressed, and remaps them to a clean `[0.0 .. 1.0]` logical range.
    * Other axes remain in standard `[-1.0 .. 1.0]` range.

* **Dynamic function helpers**

    * Automatically generates methods from the loaded config:

        * **Axis functions:**
          Call `node.<function_name>(value)` to deflect an axis.
          Example: `node.drive(0.5)`
        * **Button functions:**

            * `press_<function_name>()` → one-shot press
            * `hold_<function_name>()` → hold button until released
            * `release_<function_name>()` → release held button
              Example:

              ```python
              node.press_flipper_back_up()
              node.hold_flipper_back_up()
              node.release_flipper_back_up()
              ```

* **Safe input mapping**

    * Validates that held/pressed/deflected inputs exist in the current active mode.
    * Prevents impossible combinations across multiple modes.

---

## Topics

| Direction | Topic                           | Type                  | Description                                      |
| --------- | ------------------------------- | --------------------- | ------------------------------------------------ |
| **Pub**   | `<ocs_namespace>/joy`           | `sensor_msgs/msg/Joy` | Emulated joystick output                         |
| **Sub**   | `<ocs_namespace>/active_config` | `std_msgs/msg/String` | Reports current active mode from gamepad manager |

---

## Parameters

| Name          | Type   | Default | Description                                                                               |
| ------------- | ------ | ------- | ----------------------------------------------------------------------------------------- |
| `joy_rate_hz` | double | `50.0`  | Publish rate for Joy messages                                                             |
| *(internal)*  | —      | —       | `config_name` and `ocs_namespace` are fetched from the gamepad manager node automatically |

---


### Controlling from code

```python
from fake_joy_publisher.fake_joy_publisher import FakeJoyPublisher
import rclpy

rclpy.init()
node = FakeJoyPublisher()

# Axis control
node.drive(0.5)  # 50% forward

# Button presses
node.press_flipper_back_down()
node.hold_flipper_back_down()
node.release_flipper_back_down()

rclpy.spin(node)
```

---

## Example: Trigger Axis Mapping

* **Raw hardware values:**
  LT/RT = `1.0` (neutral) → `-1.0` (fully pressed)
* **Logical range in Fake Joy Publisher:**
  LT/RT = `0.0` (neutral) → `1.0` (fully pressed)

Example:

```python
node.lt_trigger(0.0)  # neutral
node.lt_trigger(1.0)  # fully pressed
```

---

## When to use

* Automated regression tests for gamepad-controlled robots.

