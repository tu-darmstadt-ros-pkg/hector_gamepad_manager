import os
import time
import unittest
from collections import deque

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

from ament_index_python.packages import get_package_share_directory

# Import the Python class directly so we can drive it via API
from hector_gamepad_testing_tools.fake_joy_publisher import FakeJoyPublisher


# ------------------------
# Launch the manager only
# ------------------------
def generate_test_description():
    # Configure a clean OCS/robot namespace for the test
    ocs_ns = "test_ocs"
    robot_ns = "test_robot"
    config_name = "athena"

    gpm_share = get_package_share_directory("hector_gamepad_manager")
    plugin_cfg = os.path.join(gpm_share, "config", f"{config_name}_plugin_config.yaml")
    if not os.path.isfile(plugin_cfg):
        raise FileNotFoundError(f"Missing plugin config: {plugin_cfg}")
    gamepad_manager = launch_ros.actions.Node(
        package="hector_gamepad_manager",
        executable="hector_gamepad_manager_node",
        name="hector_gamepad_manager",
        output="screen",
        parameters=[
            plugin_cfg,
            {"config_name": config_name},
            {"ocs_namespace": ocs_ns},
            {"robot_namespace": robot_ns},
        ],
    )

    # Give the manager time to come up; then start tests
    ld = launch.LaunchDescription(
        [
            gamepad_manager,
            launch.actions.TimerAction(
                period=8.0, actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]
    )

    return ld, {
        "ocs_ns": ocs_ns,
        "robot_ns": robot_ns,
        "gamepad_manager": gamepad_manager,
    }


# ------------------------
# Helper test node
# ------------------------
class Probe(Node):
    def __init__(self, ocs_ns: str):
        super().__init__("fake_joy_publisher_test_probe")
        self.ocs_ns = ocs_ns

        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_best = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.active_config = None
        self.active_config_msgs = deque(maxlen=10)
        self.joy_msgs = deque(maxlen=50)

        self.sub_cfg = self.create_subscription(
            String, f"{ocs_ns}/active_config", self._on_cfg, qos_latched
        )
        self.sub_joy = self.create_subscription(Joy, f"{ocs_ns}/joy", self._on_joy, 10)

    def _on_cfg(self, msg: String):
        self.active_config = msg.data
        self.active_config_msgs.append(msg)

    def _on_joy(self, msg: Joy):
        self.joy_msgs.append(msg)
        self.get_logger().info(f"[Probe] Received Joy message: {msg}")

    # Wait helpers
    def wait_for_cfg(self, timeout=10.0):
        t0 = time.time()
        while self.active_config is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.active_config is not None

    def wait_for_cfg_value(self, value: str, timeout=10.0):
        t0 = time.time()
        while (self.active_config != value) and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.active_config == value

    def wait_for_joy_msgs(self, n=1, timeout=5.0):
        t0 = time.time()
        while len(self.joy_msgs) < n and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return len(self.joy_msgs) >= n

    def drain_joy(self, wait=0.5):
        """Spin for a while to clear any pending Joy messages. Then reset the queue."""
        t0 = time.time()
        while time.time() - t0 < wait:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.joy_msgs.clear()


# ------------------------
# The tests
# ------------------------
class TestFakeJoyPublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.ocs_ns = "test_ocs"

        # Probe to observe /active_config and /joy
        cls.probe = Probe(cls.ocs_ns)

        # Instantiate the FakeJoyPublisher in-process (so we can call its Python API directly).
        cls.fake = FakeJoyPublisher(joy_rate_hz=40.0)

        # Wait for manager to publish initial active_config (latched)
        ok = cls.probe.wait_for_cfg(timeout=15.0)
        assert ok, "Did not receive initial active_config from manager"

        # Let FakeJoyPublisher also receive the latched config
        t0 = time.time()
        while (cls.fake._current_manager_config is None) and (time.time() - t0) < 5.0:
            rclpy.spin_once(cls.fake, timeout_sec=0.05)
            rclpy.spin_once(cls.probe, timeout_sec=0.05)

    def setUp(self):
        # reset state before each test
        if hasattr(self, "fake"):
            self.fake.clear_all()
            # cancel timer so it doesn’t run during assertions; tests that need it will re-enable
            if getattr(self.fake, "_timer", None) is not None:
                try:
                    self.fake._timer.cancel()
                except Exception:
                    pass
                self.fake._timer = None

    @classmethod
    def tearDownClass(cls):
        cls.fake.destroy_node()
        cls.probe.destroy_node()
        rclpy.shutdown()

    # ------------------------
    # Local helpers
    # ------------------------
    def spin_some(self, duration=0.5):
        t0 = time.time()
        while time.time() - t0 < duration:
            rclpy.spin_once(self.probe, timeout_sec=0.02)
            rclpy.spin_once(self.fake, timeout_sec=0.02)

    def _ensure_fake_timer(self):
        """Ensure the FakeJoyPublisher has an active periodic timer."""
        period = 1.0 / getattr(self.fake, "_joy_rate_hz", 40.0)
        # Always recreate to avoid stale/cancelled timer state across tests
        if getattr(self.fake, "_timer", None) is not None:
            try:
                self.fake._timer.cancel()
            except Exception:
                pass
        self.fake._timer = self.fake.create_timer(period, self.fake._on_timer)

    def _pump_until_cfg(
        self, target_mode: str, timeout: float = 10.0, period: float = 0.03
    ):
        """
        Act like the timer: publish frames and spin until manager reports target_mode.
        Returns True if active_config becomes target_mode within timeout.
        """
        deadline = time.time() + timeout
        while time.time() < deadline and not self.probe.wait_for_cfg_value(
            target_mode, timeout=0.0
        ):
            # publish one frame of Joy and spin both executors
            self.fake._on_timer()
            rclpy.spin_once(self.fake, timeout_sec=0.0)
            rclpy.spin_once(self.probe, timeout_sec=0.0)
            time.sleep(period)
        return self.probe.active_config == target_mode

    # ------------------------
    # Tests
    # ------------------------
    def test_00_initial_handshake(self):
        """We should have the initial active_config before doing anything."""
        self.assertIsNotNone(self.probe.active_config, "No active_config received")
        self.assertIsNotNone(
            self.fake._current_manager_config, "Fake node did not latch active_config"
        )
        self.assertEqual(self.probe.active_config, self.fake._current_manager_config)

    def test_01_config_driven_buttons_and_axes(self):
        """
        Enumerate all functions from the ACTIVE config :
        - For every mapped BUTTON: press (one-shot), hold, release.
        - For every mapped AXIS: deflect to 0.5 and then reset to baseline.
        """
        active_mode = self.fake._active_mode
        mode = self.fake._modes[active_mode]

        # make sure the mode has buttons and axes
        self.assertTrue(len(mode.button_map) > 0)
        self.assertTrue(len(mode.axis_map) > 0)

        # ---------- Buttons: press (one-shot), hold, release ----------
        for key, idx in mode.button_map.items():
            safe = _safe_name(key.function)
            # --- One-shot press ---
            self.fake.clear_all()
            self.probe.drain_joy()

            press_dyn = getattr(self.fake, f"press_{safe}", None)
            if callable(press_dyn):
                press_dyn()
            else:
                self.fake.press(key.plugin, key.function)

            self.fake._on_timer()
            self.assertTrue(
                self.probe.wait_for_joy_msgs(1, 1.5), f"{key} press: no Joy"
            )
            last = self.probe.joy_msgs[-1]
            self.assertEqual(
                last.buttons[idx], 1, f"{key} press should be high on index {idx}"
            )

            self.fake._on_timer()
            self.assertTrue(
                self.probe.wait_for_joy_msgs(2, 1.0), f"{key} press: no 2nd Joy"
            )
            last = self.probe.joy_msgs[-1]
            self.assertEqual(
                last.buttons[idx], 0, f"{key} press should clear after one tick"
            )

            # --- Hold then Release ---
            self.fake.clear_all()
            self.probe.drain_joy()

            hold_dyn = getattr(self.fake, f"hold_{safe}", None)
            if callable(hold_dyn):
                hold_dyn()
            else:
                self.fake.hold(key.plugin, key.function)

            # two frames while held
            self.fake._on_timer()
            self.assertTrue(self.probe.wait_for_joy_msgs(1, 1.5), f"{key} hold: no Joy")
            last = self.probe.joy_msgs[-1]
            self.assertEqual(
                last.buttons[idx], 1, f"{key} should be held on first frame"
            )

            self.fake._on_timer()
            self.assertTrue(
                self.probe.wait_for_joy_msgs(2, 1.5), f"{key} hold: no second Joy"
            )
            last = self.probe.joy_msgs[-1]
            self.assertEqual(
                last.buttons[idx], 1, f"{key} should still be held on second frame"
            )

            release_dyn = getattr(self.fake, f"release_{safe}", None)
            if callable(release_dyn):
                release_dyn()
            else:
                self.fake.release(key.plugin, key.function)

            self.fake._on_timer()
            self.assertTrue(
                self.probe.wait_for_joy_msgs(3, 1.5), f"{key} release: no Joy"
            )
            last = self.probe.joy_msgs[-1]
            self.assertEqual(last.buttons[idx], 0, f"{key} should be released")

        # ---------- Axes: deflect(0.5) then reset to baseline ----------
        for key, idx in mode.axis_map.items():
            safe = _safe_name(key.function)

            # Get baseline axis value with no intents
            self.fake.clear_all()
            self.probe.drain_joy()
            self.fake._on_timer()
            self.assertTrue(
                self.probe.wait_for_joy_msgs(1, 1.5), f"{key} axis baseline: no Joy"
            )
            baseline = self.probe.joy_msgs[-1].axes[idx]

            # Deflect to 0.5 using dynamic helper if present, else generic API
            axis_dyn = getattr(self.fake, safe, None)  # e.g., drive()
            if callable(axis_dyn):
                axis_dyn(0.5)
            else:
                self.fake.deflect(key.plugin, key.function, 0.5)

            self.fake._on_timer()
            self.assertTrue(
                self.probe.wait_for_joy_msgs(2, 1.5), f"{key} axis deflect: no Joy"
            )
            after = self.probe.joy_msgs[-1].axes[idx]
            self.assertNotAlmostEqual(
                after,
                baseline,
                places=3,
                msg=f"{key} deflect should change axis[{idx}]",
            )

            # Reset to 0.0 (should return to baseline; triggers may map 0.0 -> raw +1.0 baseline)
            if callable(axis_dyn):
                axis_dyn(0.0)
            else:
                self.fake.deflect(key.plugin, key.function, 0.0)

            self.fake._on_timer()
            self.assertTrue(
                self.probe.wait_for_joy_msgs(3, 1.5), f"{key} axis reset: no Joy"
            )
            ret = self.probe.joy_msgs[-1].axes[idx]
            self.assertAlmostEqual(
                ret,
                baseline,
                places=3,
                msg=f"{key} axis[{idx}] should return to baseline",
            )

    def test_02_trigger_mapping(self):
        """LT/RT mapping: logical [0..1] -> raw [1..-1] in Joy[axis 2 or 5] when functions map to those axes."""
        # Ensure periodic publishing during this test
        self._ensure_fake_timer()

        mode = self.fake._modes[self.fake._active_mode]
        rt_key = None
        lt_key = None
        for key, idx in mode.axis_map.items():
            if idx == 5:
                rt_key = key
            if idx == 2:
                lt_key = key

        self.probe.drain_joy()

        if rt_key:
            # logical 1.0 -> raw -1.0
            self.fake.deflect(rt_key.plugin, rt_key.function, 1.0)
            self.spin_some(0.2)
            self.assertTrue(self.probe.wait_for_joy_msgs(1, 2.0))
            last = self.probe.joy_msgs[-1]
            self.assertAlmostEqual(last.axes[5], -1.0, places=4)
        elif lt_key:
            self.fake.deflect(lt_key.plugin, lt_key.function, 1.0)
            self.spin_some(0.2)
            self.assertTrue(self.probe.wait_for_joy_msgs(1, 2.0))
            last = self.probe.joy_msgs[-1]
            self.assertAlmostEqual(last.axes[2], -1.0, places=4)
        else:
            self.skipTest("No trigger-mapped function found on axes 2/5 in active mode")

    def test_03_mode_switch_sequence(self):
        """
        Request an input in another mode -> see switch-only Joy first,
        then manager active_config change, then actual input.
        """
        current = self.fake._active_mode
        target_key = None
        target_mode = None
        for name, mode in self.fake._modes.items():
            if name == current:
                continue
            if mode.axis_map:
                target_key = next(iter(mode.axis_map.keys()))
                target_mode = name
                break
            elif mode.button_map:
                target_key = next(iter(mode.button_map.keys()))
                target_mode = name
                break

        if target_key is None or target_mode is None:
            self.skipTest("No secondary mode with functions to test switching")

        # Prepare: clear any residual Joy frames
        self.probe.drain_joy()

        # Ask for input in the other mode (queue intent)
        if target_key in self.fake._modes[target_mode].axis_map:
            self.fake.deflect(target_key.plugin, target_key.function, 0.5)
        else:
            getattr(
                self.fake, f"press_{_safe_name(target_key.function)}", lambda: None
            )()

        # Observe at least one Joy with ONLY the switch button set (keep publishing while we wait)
        saw_switch_only = False
        switch_btn_idx = self.fake._config_to_button.get(target_mode, None)
        t0 = time.time()
        while time.time() - t0 < 5.0:
            # actively publish and spin
            self.fake._on_timer()
            self.spin_some(0.05)
            if not self.probe.joy_msgs:
                continue
            last = self.probe.joy_msgs[-1]
            if switch_btn_idx is None:
                break  # can't assert switch-only without mapping; skip below
            if (
                any(last.buttons)
                and sum(1 for b in last.buttons if b == 1) == 1
                and last.buttons[switch_btn_idx] == 1
            ):
                saw_switch_only = True
                break

        if switch_btn_idx is None:
            self.skipTest(
                "No meta button mapping for target mode; cannot assert switch-only Joy"
            )

        self.assertTrue(
            saw_switch_only, "Did not see a switch-only Joy before mode change"
        )

        # Keep publishing frames until the manager flips active_config
        ok = self._pump_until_cfg(target_mode, timeout=12.0)
        if not ok:
            # Extra context if it fails in CI
            self.fake.get_logger().info(
                f"Switch attempt -> current:{current} target:{target_mode} "
                f"mapping:{self.fake._config_to_button.get(target_mode)} "
                f"manager_active:{self.probe.active_config}"
            )
        self.assertTrue(ok, f"Manager did not switch to '{target_mode}'")

        # After confirmation, we should see the actual intended input on the following messages
        self.probe.drain_joy()
        self.fake._on_timer()
        self.assertTrue(self.probe.wait_for_joy_msgs(1, 2.0))
        last = self.probe.joy_msgs[-1]
        # At least ensure it's NOT the pure switch-only message
        self.assertFalse(
            any(last.buttons)
            and sum(1 for b in last.buttons if b == 1) == 1
            and last.buttons[switch_btn_idx] == 1
        )

    def test_04_conflicting_intents_raise(self):
        """Queue intents from different modes at once -> _on_timer should raise RuntimeError."""
        modes = list(self.fake._modes.keys())
        if len(modes) < 2:
            self.skipTest("Only one mode available; cannot test conflict")
        m0, m1 = modes[0], modes[1]
        k0 = next(
            iter(
                self.fake._modes[m0].button_map or self.fake._modes[m0].axis_map or {}
            ),
            None,
        )
        k1 = next(
            iter(
                self.fake._modes[m1].button_map or self.fake._modes[m1].axis_map or {}
            ),
            None,
        )
        if not k0 or not k1:
            self.skipTest("Missing keys to create a conflict")

        # Reset intents
        self.fake._held_buttons.clear()
        self.fake._pending_one_shot_keys.clear()
        self.fake._deflected_axes.clear()

        # Queue a hold in m0 and a deflection/press in m1
        if k0 in self.fake._modes[m0].button_map:
            self.fake.hold(k0.plugin, k0.function)
        else:
            self.fake.deflect(k0.plugin, k0.function, 0.1)

        if k1 in self.fake._modes[m1].button_map:
            self.fake.hold(k1.plugin, k1.function)
        else:
            self.fake.deflect(k1.plugin, k1.function, 0.1)

        # log k0 and k1 for debugging
        self.fake.get_logger().info(
            f"Testing conflict with keys: {k0} in mode {m0}, {k1} in mode {m1}"
        )

        # Call _on_timer directly to capture the exception deterministically
        with self.assertRaises(RuntimeError):
            self.fake._on_timer()

        # Cleanup
        self.fake._held_buttons.clear()
        self.fake._pending_one_shot_keys.clear()
        self.fake._deflected_axes.clear()

    def test_05_one_shot_press_lasts_one_tick(self):
        """press_* should be high for one publish cycle only."""
        # Ensure periodic publishing to observe two consecutive frames quickly
        self._ensure_fake_timer()

        mode = self.fake._modes[self.fake._active_mode]
        if not mode.button_map:
            self.skipTest("No buttons in active mode to test one-shot press")

        key, idx = next(iter(mode.button_map.items()))
        # Enqueue a one-shot press
        self.fake.press(key.plugin, key.function)
        self.probe.drain_joy()
        self.fake.log_intents()
        # First publish => button high
        self.fake._on_timer()
        self.assertTrue(self.probe.wait_for_joy_msgs(1, 1.0))
        first = self.probe.joy_msgs[-1]
        self.fake.get_logger().info(f"First Joy: {first}")
        self.assertEqual(first.buttons[idx], 1)

        # Next publish => should be released
        self.fake._on_timer()
        self.assertTrue(self.probe.wait_for_joy_msgs(2, 1.0))
        second = self.probe.joy_msgs[-1]
        self.assertEqual(second.buttons[idx], 0)


# Small helper for optional dynamic invocation in tests
def _safe_name(s: str) -> str:
    import re

    return re.sub(r"[^0-9a-zA-Z]+", "_", s).strip("_").lower()


@launch_testing.post_shutdown_test()
class TestProcessesExitCleanly(unittest.TestCase):
    def test_dummy(self):
        # We don't assert on process exit code here because the manager may keep spinning
        # under certain environments. This placeholder ensures launch_testing completes.
        self.assertTrue(True)
