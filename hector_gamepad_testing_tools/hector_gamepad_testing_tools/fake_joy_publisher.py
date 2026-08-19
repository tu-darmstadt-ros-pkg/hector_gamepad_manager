#!/usr/bin/env python3
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, Set, FrozenSet
import time
import re

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType as PT
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import yaml
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class JoyLayout:
    axis_count: int
    button_count: int
    neutrals: Tuple[float, ...]
    trigger_axes: FrozenSet[int]

    def neutral_axes(self) -> list[float]:
        return list(self.neutrals)

    def logical_to_raw(self, idx: int, val: float) -> float:
        # apply range clamping -> [-1, 1] for all axes
        val = max(-1.0, min(1.0, val))
        # Triggers are logically 0 (released) to 1 (pressed) but go out on the wire negated.
        return -val if idx in self.trigger_axes else val

    @classmethod
    def game_controller(cls) -> "JoyLayout":
        """What `game_controller_node` publishes: SDL's canonical GameController layout.

        Every axis rests at 0 and the d-pad is four real buttons. A fully pressed trigger goes out
        as -1; the manager flips it back to 0..1 (see isTriggerAxis in gamepad_buttons.hpp).
        """
        return cls(
            axis_count=6,
            button_count=21,  # physical buttons on the wire; virtual buttons are axis-derived
            neutrals=tuple(0.0 for _ in range(6)),
            trigger_axes=frozenset(
                {AXIS_IDS["left_trigger"], AXIS_IDS["right_trigger"]}
            ),
        )


# Canonical name -> wire index, mirroring hector_gamepad_manager's gamepad_buttons.hpp. Configs
# are keyed by these names; the indices only exist on the Joy message.
BUTTON_IDS: Dict[str, int] = {
    "a": 0,
    "b": 1,
    "x": 2,
    "y": 3,
    "back": 4,
    "guide": 5,
    "start": 6,
    "left_stick_click": 7,
    "right_stick_click": 8,
    "left_bumper": 9,
    "right_bumper": 10,
    "dpad_up": 11,
    "dpad_down": 12,
    "dpad_left": 13,
    "dpad_right": 14,
    "share": 15,
    "paddle1": 16,
    "paddle2": 17,
    "paddle3": 18,
    "paddle4": 19,
    "touchpad": 20,
}

AXIS_IDS: Dict[str, int] = {
    "left_stick_x": 0,
    "left_stick_y": 1,
    "right_stick_x": 2,
    "right_stick_y": 3,
    "left_trigger": 4,
    "right_trigger": 5,
}


def _button_index(name: str, where: str) -> int:
    if name not in BUTTON_IDS:
        raise ValueError(f"Unknown button '{name}' in {where}")
    return BUTTON_IDS[name]


def _axis_index(name: str, where: str) -> int:
    if name not in AXIS_IDS:
        raise ValueError(f"Unknown axis '{name}' in {where}")
    return AXIS_IDS[name]


# Config "axis_buttons" name -> (axis index, logical deflection). Pressing such a virtual button
# means deflecting the axis past the manager's press threshold; there is no wire button for it.
AXIS_BUTTON_TO_AXIS: Dict[str, Tuple[int, float]] = {
    "left_stick_left": (0, 1.0),
    "left_stick_right": (0, -1.0),
    "left_stick_up": (1, 1.0),
    "left_stick_down": (1, -1.0),
    "right_stick_left": (2, 1.0),
    "right_stick_right": (2, -1.0),
    "right_stick_up": (3, 1.0),
    "right_stick_down": (3, -1.0),
    "left_trigger": (4, 1.0),
    "right_trigger": (5, 1.0),
    # The d-pad is not here: SDL reports it as four real buttons, so it needs no deflection.
}


@dataclass(frozen=True)
class Key:
    plugin: str
    function: str


@dataclass
class Mapping:
    button_idx: Optional[int] = None
    axis_idx: Optional[int] = None


@dataclass
class Mode:
    name: str
    button_map: Dict[Key, int]
    axis_map: Dict[Key, int]
    # virtual (axis-derived) buttons: Key -> axis_buttons name (see AXIS_BUTTON_TO_AXIS)
    virtual_button_map: Dict[Key, str]


def _pkg_config_path(pkg: str, name: str) -> str:
    base = get_package_share_directory(pkg)
    filename = name if name.endswith(".yaml") else f"{name}.yaml"
    return f"{base}/config/{filename}"


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _safe_func_name(name: str) -> str:
    """Map a function string to a safe, lowercase method identifier."""
    return re.sub(r"[^0-9a-zA-Z]+", "_", name).strip("_").lower()


def _request_params_sync(
    node: Node, target_node_fqn: str, names: Tuple[str, ...]
) -> Dict[str, Optional[str]]:
    """
    Fetch parameter values from another node. Returns strings or None if not set/not string.
    """
    service = f"{target_node_fqn}/get_parameters"
    client = node.create_client(GetParameters, service)
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"Parameter service not available: {service}")

        req = GetParameters.Request()
        req.names = list(names)
        fut = client.call_async(req)

        # An executor bound to this node's context, rather than rclpy's global one. The global
        # executor belongs to the default context, so on a node built with `context=` - which
        # is how a test harness runs one - spinning it here would wait on a context that has
        # nothing to do with this node, and the call would time out with the service sitting
        # right there.
        executor = SingleThreadedExecutor(context=node.context)
        try:
            rclpy.spin_until_future_complete(node, fut, executor, timeout_sec=5.0)
        finally:
            executor.shutdown()

        if not fut.done() or fut.result() is None:
            raise RuntimeError(f"Failed to get parameters from {target_node_fqn}")

        out: Dict[str, Optional[str]] = {}
        for n, v in zip(names, fut.result().values):
            if v.type == PT.PARAMETER_STRING:
                out[n] = v.string_value
            else:
                out[n] = None  # treat NOT_SET / wrong type as absent
        return out
    finally:
        # The client outlives this call otherwise, and a fake gamepad that is created once per
        # test leaves one behind every time
        node.destroy_client(client)


def _rank_manager_candidate(my_ns: str, basename: str, name: str, ns: str) -> tuple:
    """
    Higher tuple = better candidate. Preference:
    1) Same namespace
    2) Name startswith basename
    3) Name contains basename
    4) Shorter extra suffix
    """
    same_ns = int(ns == my_ns)
    starts = int(name.startswith(basename))
    contains = int(basename in name)
    # extra suffix length if startswith, else length penalty
    extra_len = len(name) - len(basename) if starts else len(name)
    return (same_ns, starts, contains, -(1000 - min(extra_len, 1000)))


def _find_manager_fqn(node: Node, basename: str, timeout_sec: float = 5.0) -> str:
    """
    Find a node whose name includes `basename` (e.g., 'hector_gamepad_manager').
    Prefer same namespace + startswith. Returns fully-qualified name '/ns/name'.
    """
    deadline = time.time() + timeout_sec
    my_ns = node.get_namespace()
    while time.time() < deadline:
        candidates = node.get_node_names_and_namespaces()
        ranked = []
        for name, ns in candidates:
            if basename in name:
                ranked.append(
                    (_rank_manager_candidate(my_ns, basename, name, ns), name, ns)
                )
        if ranked:
            ranked.sort(reverse=True)
            _, name, ns = ranked[0]
            fqn = f"{ns.rstrip('/')}/{name}".replace("//", "/")
            return fqn
        time.sleep(0.1)
    # no match: provide diagnostics
    all_nodes = ", ".join(
        [
            f"{ns.rstrip('/')}/{name}".replace("//", "/")
            for name, ns in node.get_node_names_and_namespaces()
        ]
    )
    raise RuntimeError(
        f"Could not find a node containing '{basename}' in its name. "
        f"Seen nodes: {all_nodes}"
    )


def _keys_in_mode_axis(keys: Set["Key"], mode: "Mode") -> Set["Key"]:
    return {k for k in keys if k in mode.axis_map}


def _keys_in_mode_button(keys: Set["Key"], mode: "Mode") -> Set["Key"]:
    return {k for k in keys if k in mode.button_map or k in mode.virtual_button_map}


class FakeJoyPublisher(Node):
    """
    Emulates a gamepad by publishing Joy messages and auto-switching active config
    based on hector_gamepad_manager meta-config.

    Public API:
      press(plugin, function)
      hold(plugin, function)
      release(plugin, function)
      deflect(plugin, function, value)
        - For left_trigger/right_trigger: value in [0,1] (0 = released, 1 = fully pressed)
        - For others:                     value in [-1,1]

    Timer control:
      start_publishing(rate_hz: float | None = None)
      stop_publishing()
      is_publishing() -> bool
      set_publish_rate(rate_hz: float)
      publish_once() / tick()

    Dynamic helpers (added via __getattr__):
      - <axis_func>(value)          -> deflect(plugin, func, value)
      - press_<button_func>()       -> press(plugin, func)
      - hold_<button_func>()        -> hold(plugin, func)
      - release_<button_func>()     -> release(plugin, func)
    """

    def __init__(
        self,
        node_name: str = "fake_joy_publisher",
        manager_node_basename: str = "hector_gamepad_manager",
        joy_rate_hz: float = 50.0,
        layout: Optional[JoyLayout] = None,
        context: Optional[rclpy.context.Context] = None,
        use_sim_time: Optional[bool] = None,
    ):
        """
        :param context: The rclpy context to build this node in. Needed to run inside a test
            harness that keeps a context of its own - `better_launch_testing` does, so that the
            system under test cannot wedge the test's own node:

                gamepad = env.attach_node(FakeJoyPublisher(context=env.context))

            None uses the default context, which is what a standalone run wants.
        :param use_sim_time: Set the node's `use_sim_time` parameter. In simulation the Joy
            messages should be stamped on the same clock as everything they cause, or a test
            comparing a gamepad press with what the robot did afterwards is subtracting two
            different clocks.
        """
        super().__init__(node_name, context=context)
        self.layout = layout or JoyLayout.game_controller()
        self.declare_parameter("joy_rate_hz", float(joy_rate_hz))
        self._joy_rate_hz = float(self.get_parameter("joy_rate_hz").value)

        if use_sim_time is not None:
            self.set_parameters(
                [rclpy.parameter.Parameter("use_sim_time", value=bool(use_sim_time))]
            )

        # ---- Find the actual manager node FQN (handles random suffixes) ----
        manager_fqn = _find_manager_fqn(self, manager_node_basename)

        # ---- Pull config_name from that node ----
        params = _request_params_sync(self, manager_fqn, ("config_name",))
        config_name = params.get("config_name") or "athena"
        # The manager runs in the robot namespace; joy and joy_teleop_profile live there. Derive
        # that namespace from the manager node's FQN ('/<robot_ns>/<node_name>').
        robot_ns = manager_fqn.rsplit("/", 1)[0] or "/"
        self.get_logger().info(
            f"Using manager '{manager_fqn}' with config_name='{config_name}', robot_namespace='{robot_ns}'"
        )

        # ---- Load meta-config & modes ----
        meta = _load_yaml(_pkg_config_path("hector_gamepad_manager", config_name))
        self._default_mode_name: str = meta.get("default_config", "driving")
        self._config_switch_reserved_buttons: Set[int] = set()
        modes_to_load: Dict[str, str] = {}
        # map config_name -> button index for switch-only publish
        self._config_to_button: Dict[str, int] = {}

        for k, v in (meta.get("buttons") or {}).items():
            idx = _button_index(k, f"meta config '{config_name}'")
            pkg = (v or {}).get("package", "") or ""
            cfg = (v or {}).get("config", "") or ""
            if pkg and cfg:
                modes_to_load[cfg] = pkg
                self._config_switch_reserved_buttons.add(idx)
                self._config_to_button[cfg] = idx

        self._modes: Dict[str, Mode] = {}
        for mode_name, pkg in modes_to_load.items():
            cfg = _load_yaml(_pkg_config_path(pkg, mode_name))
            self._modes[mode_name] = self._parse_mode(
                mode_name, cfg, self._config_switch_reserved_buttons
            )

        if self._default_mode_name not in self._modes:
            try:
                cfg = _load_yaml(
                    _pkg_config_path("hector_gamepad_manager", self._default_mode_name)
                )
                self._modes[self._default_mode_name] = self._parse_mode(
                    self._default_mode_name, cfg, self._config_switch_reserved_buttons
                )
            except Exception as e:
                raise RuntimeError(
                    f"Default mode '{self._default_mode_name}' not loadable: {e}"
                )

        # ---- Build reverse indices for dynamic methods ----
        self._axis_name_to_keys: Dict[str, Set[Key]] = {}
        self._button_name_to_keys: Dict[str, Set[Key]] = {}
        self._reindex_functions()

        # ---- Track manager-reported active config & wait until received ----
        self._current_manager_config: Optional[str] = None
        qos_cfg = QoSProfile(depth=1)
        qos_cfg.reliability = ReliabilityPolicy.RELIABLE
        qos_cfg.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._active_config_sub = self.create_subscription(
            String,
            f"{robot_ns.strip('/')}/joy_teleop_profile",
            self._active_config_cb,
            qos_cfg,
        )

        # Local view of active mode (kept in sync with manager)
        self._active_mode: str = self._default_mode_name

        # Physical Joy state and logical intents
        self._axes = self.layout.neutral_axes()
        self._buttons = [0] * self.layout.button_count
        self._deflected_axes: Dict[
            Key, float
        ] = {}  # logical values (0..1 for triggers, else -1..1)
        self._held_buttons: Set[Key] = set()
        # one-shot presses to apply on publish
        self._pending_one_shot_keys: Set[Key] = set()

        # Publisher
        topic = f"{robot_ns.strip('/')}/joy"
        self._pub = self.create_publisher(Joy, topic, 10)
        self.get_logger().info(f"Publishing Joy on '{topic}' at {self._joy_rate_hz} Hz")

        # Timer (can be stopped/started via public API)
        self._timer: Optional[rclpy.timer.Timer] = None
        self._create_timer()  # start running by default

    # ---------- Public API (Inputs) ----------
    def press(self, plugin: str, function: str) -> None:
        self.get_logger().info(f"Pressing button {plugin}::{function}")
        self._pending_one_shot_keys.add(Key(plugin, function))

    def hold(self, plugin: str, function: str) -> None:
        self._held_buttons.add(Key(plugin, function))

    def release(self, plugin: str, function: str) -> None:
        """Stop supplying this input, whether it is a button or an axis.

        The axis half is not decoration: `holding()` deflects an axis and releases it in its
        `finally`, so without this a `with gamepad.holding(drive, 1.0):` block leaves the stick
        at full deflection when it exits and the robot drives on. That is invisible in a test
        that only asserts the robot moved, and it is precisely what breaks the test after it.
        """
        key = Key(plugin, function)
        self._held_buttons.discard(key)
        self._pending_one_shot_keys.discard(key)
        self._deflected_axes.pop(key, None)

    @contextmanager
    def holding(self, plugin: str, function: str, value: Optional[float] = None):
        """Hold an input for the duration of a `with` block, and let go afterwards.

        The shape most tests want, because the interesting assertions happen *while* something
        is held - and a test that fails in the middle would otherwise leave the robot driving
        into the next one.

            with gamepad.holding("drive", "drive"):
                env.expect_motion("odom", "base_link", forward=0.2)

        :param value: For an axis, how far to deflect it. Omit for a button.
        """
        if value is None:
            self.hold(plugin, function)
        else:
            self.deflect(plugin, function, value)

        try:
            yield self
        finally:
            self.release(plugin, function)

    def deflect(self, plugin: str, function: str, value: float) -> None:
        self._deflected_axes[Key(plugin, function)] = float(value)

    def clear_all(self) -> None:
        """Clear all held buttons and deflected axes and reset physical state."""
        self._held_buttons.clear()
        self._deflected_axes.clear()
        self._pending_one_shot_keys.clear()
        self._reset_physical_state()

    def log_intents(self) -> None:
        self.get_logger().info(
            f"Intents: held={self._held_buttons}, one-shots={self._pending_one_shot_keys}, deflected={self._deflected_axes}"
        )

    # ---------- Public API (Timer control) ----------
    def start_publishing(self, rate_hz: Optional[float] = None) -> None:
        """(Re)start periodic publishing. Optionally change the publish rate."""
        if rate_hz is not None:
            self.set_publish_rate(rate_hz)
            return  # set_publish_rate restarts if running
        if self._timer is None:
            self._create_timer()
        else:
            # If timer exists but was canceled, recreate to be safe
            try:
                self._timer.reset()
            except Exception:
                self._create_timer()

    def stop_publishing(self) -> None:
        """Stop periodic publishing (the node remains usable via publish_once())."""
        if self._timer is not None:
            try:
                self._timer.cancel()
            except Exception:
                pass

    def is_publishing(self) -> bool:
        """Return True if the periodic publisher timer is active."""
        return self._timer is not None and self._timer.is_canceled() is False

    def set_publish_rate(self, rate_hz: float) -> None:
        """Change publish rate. If currently publishing, restart the timer with the new rate."""
        rate_hz = float(rate_hz)
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be > 0")
        self._joy_rate_hz = rate_hz
        # Recreate timer with new period if running
        was_running = self.is_publishing()
        if self._timer is not None:
            try:
                self._timer.cancel()
            except Exception:
                pass
            self._timer = None
        if was_running:
            self._create_timer()

    def publish_once(self) -> None:
        """Publish a single Joy message by running one scheduling tick."""
        self._on_timer()

    # Short alias
    tick = publish_once

    # ---------- Dynamic method plumbing ----------
    def _reindex_functions(self) -> None:
        axis_index: Dict[str, Set[Key]] = {}
        button_index: Dict[str, Set[Key]] = {}
        for mode in self._modes.values():
            for key in mode.axis_map.keys():
                s = _safe_func_name(key.function)
                axis_index.setdefault(s, set()).add(key)
            for key in list(mode.button_map.keys()) + list(
                mode.virtual_button_map.keys()
            ):
                s = _safe_func_name(key.function)
                button_index.setdefault(s, set()).add(key)
        self._axis_name_to_keys = axis_index
        self._button_name_to_keys = button_index

    def __getattr__(self, name: str):
        # Button helpers with prefixes
        for prefix, caller in (
            ("press_", self.press),
            ("hold_", self.hold),
            ("release_", self.release),
        ):
            if name.startswith(prefix):
                fn_safe = name[len(prefix) :].lower()
                candidates = self._button_name_to_keys.get(
                    fn_safe, set()
                ) or self._button_name_to_keys.get(_safe_func_name(fn_safe), set())
                if not candidates:
                    raise AttributeError(f"No button function named '{fn_safe}' found")
                mode = self._modes.get(self._active_mode)
                if mode:
                    in_mode = _keys_in_mode_button(candidates, mode)
                    if len(in_mode) == 1:
                        key = next(iter(in_mode))
                        return lambda: caller(key.plugin, key.function)
                    elif len(in_mode) > 1:
                        raise AttributeError(
                            f"Ambiguous button '{fn_safe}' in active mode '{self._active_mode}': "
                            f"{[f'{k.plugin}::{k.function}' for k in sorted(in_mode, key=lambda x: (x.plugin, x.function))]}"
                        )
                if len(candidates) == 1:
                    key = next(iter(candidates))
                    return lambda: caller(key.plugin, key.function)
                raise AttributeError(
                    f"Ambiguous button '{fn_safe}' across modes: "
                    f"{[f'{k.plugin}::{k.function}' for k in sorted(candidates, key=lambda x: (x.plugin, x.function))]}"
                )
        # Axis helper (no prefix) -> deflect(value)
        fn_safe = _safe_func_name(name)
        candidates = self._axis_name_to_keys.get(fn_safe, set())
        if not candidates:
            raise AttributeError(f"No axis function named '{name}' found")
        mode = self._modes.get(self._active_mode)
        if mode:
            in_mode = _keys_in_mode_axis(candidates, mode)
            if len(in_mode) == 1:
                key = next(iter(in_mode))
                return lambda value: self.deflect(key.plugin, key.function, value)
            elif len(in_mode) > 1:
                raise AttributeError(
                    f"Ambiguous axis '{name}' in active mode '{self._active_mode}': "
                    f"{[f'{k.plugin}::{k.function}' for k in sorted(in_mode, key=lambda x: (x.plugin, x.function))]}"
                )
        if len(candidates) == 1:
            key = next(iter(candidates))
            return lambda value: self.deflect(key.plugin, key.function, value)
        raise AttributeError(
            f"Ambiguous axis '{name}' across modes: "
            f"{[f'{k.plugin}::{k.function}' for k in sorted(candidates, key=lambda x: (x.plugin, x.function))]}"
        )

    # ---------- Internals ----------
    def _create_timer(self) -> None:
        """Create (or recreate) the internal rclpy Timer using the current rate."""
        period = 1.0 / max(self._joy_rate_hz, 1e-6)
        # rclpy Timer can’t be restarted after cancel reliably; recreate cleanly
        self._timer = self.create_timer(period, self._on_timer)

    def _active_config_cb(self, msg: String) -> None:
        self._current_manager_config = msg.data or ""
        self._active_mode = self._current_manager_config

    def _on_timer(self) -> None:
        # If we haven't received the initial manager state yet, do nothing.
        if self._current_manager_config is None:
            return

        # Determine if a mode switch is required based on *current intents*
        intents: Set[Key] = (
            set(self._held_buttons)
            | set(self._pending_one_shot_keys)
            | set(self._deflected_axes.keys())
        )
        target_modes: Set[str] = set()

        for key in intents:
            m = self._find_mode_for_key(key, self._current_manager_config)
            if m is None:
                raise KeyError(f"{key} not found in any loaded mode")
            target_modes.add(m)

        if len(target_modes) > 1:
            # Inputs span multiple modes -> impossible configuration
            raise RuntimeError(
                f"Conflicting intents across multiple modes: {sorted(target_modes)}"
            )

        # If a switch is needed, publish a switch-only Joy and wait for confirmation
        if len(target_modes) == 1 and self._current_manager_config != next(
            iter(target_modes)
        ):
            target = next(iter(target_modes))
            if target not in self._config_to_button:
                raise RuntimeError(f"No meta button mapped for target mode '{target}'")

            switch_btn = self._config_to_button[target]
            msg_switch = Joy()
            msg_switch.header.stamp = self.get_clock().now().to_msg()
            msg_switch.axes = self.layout.neutral_axes()
            msg_switch.buttons = [0] * self.layout.button_count
            if 0 <= switch_btn < self.layout.button_count:
                msg_switch.buttons[switch_btn] = 1
            self.get_logger().info(
                f"Switching to mode '{target}' via button {switch_btn} (active config: '{self._current_manager_config}')"
            )
            self._pub.publish(msg_switch)
            return  # wait for active_config to change

        # No switch needed -> publish actual Joy
        self._active_mode = self._current_manager_config  # keep synced

        axes = self.layout.neutral_axes()
        buttons = [0] * self.layout.button_count
        mode = self._modes[self._active_mode]

        def apply_button(key: Key, action: str) -> None:
            if key in mode.button_map:
                buttons[mode.button_map[key]] = 1
            elif key in mode.virtual_button_map:
                # virtual button: deflect the corresponding axis past the press threshold
                idx, logical = AXIS_BUTTON_TO_AXIS[mode.virtual_button_map[key]]
                axes[idx] = self.layout.logical_to_raw(idx, logical)
            else:
                raise RuntimeError(
                    f"{action} button {key} not available in active mode '{self._active_mode}'"
                )

        # Apply held buttons
        for key in list(self._held_buttons):
            apply_button(key, "Held")

        # Apply one-shot presses
        for key in list(self._pending_one_shot_keys):
            self.get_logger().info(
                f"Pressing button {key} in mode '{self._active_mode}'"
            )
            apply_button(key, "Pressed")
        self._pending_one_shot_keys.clear()

        # Apply axes
        for key, logical in list(self._deflected_axes.items()):
            if key not in mode.axis_map:
                raise RuntimeError(
                    f"Axis {key} not available in active mode '{self._active_mode}'"
                )
            idx = mode.axis_map[key]
            axes[idx] = self.layout.logical_to_raw(idx, logical)

        # Publish final message
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes
        msg.buttons = buttons
        self.get_logger().info(
            f"Publishing Joy: mode='{self._active_mode}',  with buttons={buttons}, axes={axes}"
        )
        self._pub.publish(msg)

    def _parse_mode(self, name: str, cfg: dict, reserved_buttons: Set[int]) -> Mode:
        buttons = {}
        axes = {}
        virtual_buttons = {}
        for k, v in (cfg.get("buttons") or {}).items():
            idx = _button_index(k, f"config '{name}'")
            if idx in reserved_buttons:
                continue
            plugin = (v or {}).get("plugin", "") or ""
            func = (v or {}).get("function", "") or ""
            if plugin and func:
                buttons[Key(plugin, func)] = idx
        for k, v in (cfg.get("axis_buttons") or {}).items():
            if k not in AXIS_BUTTON_TO_AXIS:
                raise ValueError(f"Unknown axis button '{k}' in config '{name}'")
            plugin = (v or {}).get("plugin", "") or ""
            func = (v or {}).get("function", "") or ""
            if plugin and func:
                virtual_buttons[Key(plugin, func)] = k
        for k, v in (cfg.get("axes") or {}).items():
            idx = _axis_index(k, f"config '{name}'")
            plugin = (v or {}).get("plugin", "") or ""
            func = (v or {}).get("function", "") or ""
            if plugin and func:
                axes[Key(plugin, func)] = idx
        self.get_logger().info(
            f"Loaded mode '{name}' with {len(buttons)} buttons, "
            f"{len(virtual_buttons)} virtual buttons and {len(axes)} axes"
            f"\nButtons:{yaml.dump(buttons)}, \nVirtual buttons:{yaml.dump(virtual_buttons)}, "
            f"\nAxes:{yaml.dump(axes)}"
        )
        return Mode(
            name=name,
            button_map=buttons,
            axis_map=axes,
            virtual_button_map=virtual_buttons,
        )

    def _find_mode_for_key(self, key: Key, active_mode: str) -> Optional[str]:
        # preferred: active mode first, then all modes
        if (
            key in self._modes[active_mode].button_map
            or key in self._modes[active_mode].virtual_button_map
            or key in self._modes[active_mode].axis_map
        ):
            return active_mode
        for name, mode in self._modes.items():
            if (
                key in mode.button_map
                or key in mode.virtual_button_map
                or key in mode.axis_map
            ):
                return name
        return None

    def _reset_physical_state(self) -> None:
        self._axes = self.layout.neutral_axes()
        self._buttons = [0] * self.layout.button_count


def main():
    rclpy.init()
    node = FakeJoyPublisher()
    # Example usage of the new API:
    node.stop_publishing()  # switch to manual mode
    node.drive(0.5)  # queue an intent
    node.steer(-0.3)  # queue another intent
    node.publish_once()  # publish one tick manually
    node.start_publishing()  # resume periodic publishing at the configured rate
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
