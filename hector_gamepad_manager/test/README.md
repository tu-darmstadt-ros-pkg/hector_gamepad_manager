# Gamepad Manager Tests (rtest)

This test suite uses `rtest` to run the gamepad manager and its plugins without
spinning executors or launching ROS processes. `rtest` replaces selected `rclcpp`
entities with test doubles at compile time and records them in a static registry.
The tests then fetch those entities directly and drive the node under test by
injecting messages into subscriptions or observing published messages.

## How rtest works in this repository

1) Build-time interception
   - `rtest` provides mock versions of `rclcpp::Publisher` and
     `rclcpp::Subscription` (and other optional entities).
   - These mocks are injected at compile time via an include that is forced
     on the compiler command line. This is why the `BUILD_TESTING` path in
     CMake links `rtest::publisher_mock` and `rtest::subscription_mock` into
     the test doubles.
   - When a mocked publisher or subscription is created, it registers itself
     in `rtest::StaticMocksRegistry` under the node name and topic.

2) Lookup and control
   - `rtest::findSubscription<NodeType, MsgT>(node, topic)` returns the actual
     subscription instance for that node+topic.
   - `rtest::findPublisher<MsgT>(node, topic)` returns a `PublisherMock` that
     wraps the real publisher and exposes a mocked `publish()` method.
   - The tests call `subscription->handle_message(msg)` to inject messages
     directly into the callback without spinning.
   - The tests use `EXPECT_CALL(*publisher_mock, publish(...))` to assert
     published messages.

3) Parameters are loaded like in a normal node
   - The tests pass `--params-file test/config/athena_plugin_config.yaml` via
     `rclcpp::NodeOptions`. This replicates the launch file behavior without
     starting a separate process.

## How rtest is used in this package

The file `test/test_hector_gamepad_manager.cpp` creates a single node:
- `rclcpp::Node` is constructed with a params file.
- `hector_gamepad_manager::HectorGamepadManager` is constructed with that node.

Then the test uses rtest to get:
- `/ocs/joy` subscription to inject joystick input
- `/ocs/active_config` publisher to assert config switching
- `/athena/cmd_vel`, `/athena/moveit_twist_controller/eef_cmd`,
  `/athena/moveit_twist_controller/gripper_vel_cmd`, and
  `/athena/flipper_velocity_controller/commands` publishers to assert output
  topics from plugins.

## Deep dives: two example tests

### 1) `CmdVel` test (drive plugin)

Goal:
Verify that joystick axis input produces a valid `/athena/cmd_vel` twist
in driving mode.

Mechanics:
1) The test ensures the manager is in the "driving" config:
   - It sends a joy message with the "start" button pressed if needed.
   - This triggers the config switch logic inside the manager.
2) It retrieves the `/ocs/joy` subscription via rtest:
   - `auto sub_joy = rtest::findSubscription<sensor_msgs::msg::Joy>(node, "/ocs/joy");`
3) It retrieves the `/athena/cmd_vel` publisher mock:
   - `auto pub_cmd_vel = rtest::findPublisher<geometry_msgs::msg::TwistStamped>(node, "/athena/cmd_vel");`
4) It sets an expectation on the publisher mock:
   - The test uses `EXPECT_CALL(*pub_cmd_vel, publish(_))` to capture the
     `TwistStamped` and check the fields.
5) It injects a joystick message:
   - `sub_joy->handle_message(joy_msg);`
6) The manager forwards the input to the drive plugin. The plugin computes
   linear and angular velocities and calls `publish()` on its publisher.
7) The publisher call is intercepted by the mock and validated by the test.

Why this is reliable:
- No executor spin is needed; the subscription callback executes immediately.
- The test checks the actual `TwistStamped` data that would be published.

### 2) `SwitchConfig` test (config switching)

Goal:
Verify the active configuration changes are published on
`/ocs/active_config` in the expected sequence.

Mechanics:
1) The test fetches the `active_config` publisher mock:
   - `auto pub_config = rtest::findPublisher<std_msgs::msg::String>(node, "/ocs/active_config");`
2) It also fetches the `/ocs/joy` subscription for input injection.
3) It sets an ordered sequence of expectations:
   - First a publish with `"manipulation"`, then a publish with `"driving"`.
4) It injects a "back" button press to switch to manipulation.
5) It injects a "start" button press to switch back to driving.
6) The test verifies the exact order of published strings.

Why this is reliable:
- The config switching logic runs entirely inside the same process and is
  triggered directly by injected messages.
- `EXPECT_CALL` sequencing ensures the order is correct, not just the content.

## Notes about limitations

- `rtest` can only observe publishers/subscriptions that are compiled with the
  rtest mocks. This is why the plugin package is built with rtest mocks enabled
  under `BUILD_TESTING`.
- If multiple plugins publish to the same topic, the registry may only expose
  one. For tests, the manipulation plugin's `cmd_vel` publisher is disabled
  via `manipulation_plugin.enable_drive_cmd: false` in the test params file.

## Plugin library setup for tests

- The plugin package builds a separate `hector_gamepad_manager_plugins_test_doubles` library.
- A test-only plugin XML (`hector_gamepad_manager_plugins/plugins_test.xml`) points to that
  library and is written into the plugin build tree alongside a minimal package manifest and
  ament index entries.
- The test executable prepends the plugin build prefix to `AMENT_PREFIX_PATH`, so pluginlib
  resolves plugins from the test-doubles XML instead of the installed runtime plugins.

## Files referenced

- `test/test_hector_gamepad_manager.cpp`: test implementation using rtest
- `test/main.cpp`: gtest main that initializes and shuts down `rclcpp`
- `test/config/athena_plugin_config.yaml`: test params file
- `test/config/manager_internal_params.yaml`: test params for internal-logic tests

## Coverage

Coverage is enabled by passing `-DENABLE_COVERAGE=ON` to CMake for both
`hector_gamepad_manager` and `hector_gamepad_manager_plugins`.

Example:
```sh
colcon build --packages-up-to hector_gamepad_manager \
  --cmake-args -DBUILD_TESTING=ON -DENABLE_COVERAGE=ON

colcon test --packages-select hector_gamepad_manager \
  --event-handlers console_cohesion+

lcov --capture --directory build/hector_gamepad_manager \
  --output-file coverage.info

genhtml coverage.info --output-directory coverage_html
```
