# Fake Joy Publisher

The `fake_joy_publisher` is a ROS 2 node and Python library that emulates a gamepad.
It publishes `sensor_msgs/msg/Joy` messages as if they came from a real joystick and integrates with the `hector_gamepad_manager`.

This is primarily intended for:

* **Automated testing** of gamepad-based control.
* **Reproducibility** in CI pipelines (tests always get the same joystick state).

---

## Features

* Publishes `sensor_msgs/msg/Joy` messages at a configurable rate.
* Discovers the running `hector_gamepad_manager` node automatically (even with random suffixes).
* Loads the same plugin/mode configs as the manager, so function names map to the right axes/buttons.
* Supports **logical inputs** (high-level functions) instead of hard-coding button indices.
* Automatically **switches modes** if a requested input belongs to another config.
* Provides **dynamic methods** for convenience:

    * `node.drive(0.5)` → deflect joystick axis bound to `drive`.
    * `node.press_flipper_back_up()` → one-shot button press.
    * `node.hold_flipper_back_up()` / `node.release_flipper_back_up()` → sustained button state.
* Flexible **timer control API**: run continuously or tick manually.

---

## Public API

### Input methods

```python
node.press(plugin, function)            # one-shot button press (applied for one tick)
node.hold(plugin, function)             # hold button down
node.release(plugin, function)          # release held/pressed button
node.deflect(plugin, function, value)   # axis deflection
node.clear_all()                        # reset all inputs
```

* For **left_trigger / right_trigger**: values are in **\[0, 1]** (0 = released, 1 = fully pressed).
* For other axes: values are in **\[-1, 1]**.

### Holding an input for a block

```python
with node.holding("drive", "drive", 1.0):   # axis: pass the deflection
    ...                                     # assertions happen while it is held
with node.holding("flipper", "flipper_back_up"):   # button: omit the value
    ...
```

Releases on the way out, including when the block raises - a test that fails in the middle
would otherwise leave the robot driving into the next one.

### Timer control methods

```python
node.start_publishing(rate_hz=None)   # (re)start periodic publishing, optionally change rate
node.stop_publishing()                # stop timer (no periodic publishing)
node.is_publishing() -> bool          # check if timer is running
node.set_publish_rate(100.0)          # change publish rate (Hz), restart if needed
node.publish_once()                   # publish a single Joy message (manual tick)
node.tick()                           # alias of publish_once()
```

### Dynamic helpers

Function names from the configs are mapped automatically:

```python
node.drive(0.5)               # axis deflection
node.press_flipper_back_up()
node.hold_flipper_back_up()
node.release_flipper_back_up()
```

---

## Usage


### Python API (manual control)

```python
import rclpy
from fake_joy_publisher.fake_joy_publisher import FakeJoyPublisher

rclpy.init()
node = FakeJoyPublisher(joy_rate_hz=40.0)

# stop timer to make behavior deterministic
node.stop_publishing()

# queue inputs
node.drive(0.5)                     # axis deflection
node.press_flipper_back_up()      # one-shot button

# publish once
node.publish_once()

# resume periodic publishing if needed
node.start_publishing()

rclpy.spin(node)
```

---

## Running inside a test harness

An integration harness generally keeps an rclpy context of its own, so that the system under
test cannot wedge the test's own node. `better_launch_testing` does, and the fake gamepad is
built against it and handed over to be spun:

```python
gamepad = env.attach_node(
    FakeJoyPublisher(context=env.context, use_sim_time=True)
)
gamepad.drive(1.0)
```

* `context=` builds the node in that context. Without it the node belongs to the default
  context, which that harness's executor does not spin - the symptom is a gamepad that
  publishes nothing.
* `use_sim_time=` stamps the Joy messages on the same clock as everything they cause, which is
  what a test comparing a press against what the robot did afterwards is subtracting.

The constructor asks the manager for `config_name` over the parameter service, and spins that
call on an executor belonging to the node's own context rather than rclpy's global one - the
global executor belongs to the default context, and in a process that never called
`rclpy.init()` it does not exist at all.

## Testing

The package includes a full **test suite** (`test_fake_joy_publisher.py`) that demonstrates usage patterns:

* Switching modes by pressing config buttons.
* One-shot presses lasting exactly one tick.
* Sustained holds across ticks.
* Axis deflections for both normal axes and LT/RT triggers.
* Manual ticking (`publish_once`) for deterministic assertions.

💡 The tests are a **good reference** for writing your own scripts with the fake publisher.

---

## Notes

* Requires `hector_gamepad_manager` to be running with a valid config (YAML).
* Waits for the manager to publish its active config before publishing inputs.
* Default publish rate: 50 Hz.
