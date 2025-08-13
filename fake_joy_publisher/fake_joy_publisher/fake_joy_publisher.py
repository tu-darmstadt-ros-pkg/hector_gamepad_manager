#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple, Optional, Set, List, Callable
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType as PT
from sensor_msgs.msg import Joy
import yaml
from ament_index_python.packages import get_package_share_directory

# =========================
# Axis layout (Joy message)
# =========================
AXIS_COUNT = 8
BUTTON_COUNT = 11

AXIS_NEUTRALS = [0.0] * AXIS_COUNT
AXIS_NEUTRALS[2] = 1.0  # LT neutral
AXIS_NEUTRALS[5] = 1.0  # RT neutral

# Axes that behave like "button-style triggers" logically (0..1) but publish raw [1..-1]
SPECIAL_TRIGGER_AXES = {2, 5}

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


def _pkg_config_path(pkg: str, name: str) -> str:
    base = get_package_share_directory(pkg)
    filename = name if name.endswith(".yaml") else f"{name}.yaml"
    return f"{base}/config/{filename}"

def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def _request_params_sync(node: Node, target_node_fqn: str, names: Tuple[str, ...]) -> Dict[str, Optional[str]]:
    """
    Fetch parameter values from another node. Returns strings or None if not set/not string.
    """
    service = f"{target_node_fqn}/get_parameters"
    client = node.create_client(GetParameters, service)
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"Parameter service not available: {service}")

    req = GetParameters.Request()
    req.names = list(names)
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    if not fut.done() or fut.result() is None:
        raise RuntimeError(f"Failed to get parameters from {target_node_fqn}")

    out: Dict[str, Optional[str]] = {}
    for n, v in zip(names, fut.result().values):
        if v.type == PT.PARAMETER_STRING:
            out[n] = v.string_value
        else:
            out[n] = None  # treat NOT_SET / wrong type as absent
    return out

def _rank_manager_candidate(
        my_ns: str, basename: str, name: str, ns: str
) -> tuple:
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
    return (same_ns, starts, contains, - (1000 - min(extra_len, 1000)))

def _find_manager_fqn(node: Node, basename: str, timeout_sec: float = 5.0) -> str:
    """
    Find a node whose name includes `basename` (e.g., 'hector_gamepad_manager').
    Prefer same namespace + startswith. Returns fully-qualified name '/ns/name'.
    """
    deadline = time.time() + timeout_sec
    my_ns = node.get_namespace()
    best = None
    while time.time() < deadline:
        candidates = node.get_node_names_and_namespaces()
        ranked = []
        for name, ns in candidates:
            if basename in name:
                ranked.append(( _rank_manager_candidate(my_ns, basename, name, ns), name, ns ))
        if ranked:
            ranked.sort(reverse=True)
            _, name, ns = ranked[0]
            fqn = f"{ns.rstrip('/')}/{name}".replace("//", "/")
            return fqn
        time.sleep(0.1)
    # no match: provide diagnostics
    all_nodes = ", ".join([f"{ns.rstrip('/')}/{name}".replace('//','/') for name, ns in node.get_node_names_and_namespaces()])
    raise RuntimeError(
        f"Could not find a node containing '{basename}' in its name. "
        f"Seen nodes: {all_nodes}"
    )


class FakeJoyPublisher(Node):
    """
    Emulates a gamepad by publishing Joy messages and auto-switching active config
    based on hector_gamepad_manager meta-config.

    Public API:
      press(plugin, function)
      hold(plugin, function)
      release(plugin, function)
      deflect(plugin, function, value)
        - For LT/RT (axes 2,5): value in [0,1]  (0 = neutral, 1 = fully pressed)
        - For others:           value in [-1,1]
    """

    def __init__(self,
                 node_name: str = "fake_joy_publisher",
                 manager_node_basename: str = "hector_gamepad_manager",
                 joy_rate_hz: float = 50.0):
        super().__init__(node_name)

        self.declare_parameter("joy_rate_hz", float(joy_rate_hz))
        self._joy_rate_hz = float(self.get_parameter("joy_rate_hz").value)

        # ---- Find the actual manager node FQN (handles random suffixes) ----
        manager_fqn = _find_manager_fqn(self, manager_node_basename)

        # ---- Pull config_name & ocs_namespace from that node ----
        params = _request_params_sync(self, manager_fqn, ("config_name", "ocs_namespace"))
        config_name = params.get("config_name") or "athena"
        ocs_ns = params.get("ocs_namespace") or "ocs"
        self.get_logger().info(f"Using manager '{manager_fqn}' with config_name='{config_name}', ocs_namespace='{ocs_ns}'")

        # ---- Load meta-config & modes ----
        meta = _load_yaml(_pkg_config_path("hector_gamepad_manager", config_name))
        self._default_mode_name: str = meta.get("default_config", "driving")
        self._config_switch_reserved_buttons: Set[int] = set()
        modes_to_load: Dict[str, str] = {}
        # NEW: map config_name -> button index for switch-only publish
        self._config_to_button: Dict[str, int] = {}

        for k, v in (meta.get("buttons") or {}).items():
            idx = int(k)
            pkg = (v or {}).get("package", "") or ""
            cfg = (v or {}).get("config", "") or ""
            if pkg and cfg:
                modes_to_load[cfg] = pkg
                self._config_switch_reserved_buttons.add(idx)
                self._config_to_button[cfg] = idx  # NEW

        self._modes: Dict[str, Mode] = {}
        for mode_name, pkg in modes_to_load.items():
            cfg = _load_yaml(_pkg_config_path(pkg, mode_name))
            self._modes[mode_name] = self._parse_mode(mode_name, cfg, self._config_switch_reserved_buttons)

        if self._default_mode_name not in self._modes:
            try:
                cfg = _load_yaml(_pkg_config_path("hector_gamepad_manager", self._default_mode_name))
                self._modes[self._default_mode_name] = self._parse_mode(
                    self._default_mode_name, cfg, self._config_switch_reserved_buttons
                )
            except Exception as e:
                raise RuntimeError(f"Default mode '{self._default_mode_name}' not loadable: {e}")

        self._active_mode: str = self._default_mode_name

        # Physical Joy state and logical intents
        self._axes = self._neutral_axes()
        self._buttons = [0] * BUTTON_COUNT
        self._deflected_axes: Dict[Key, float] = {}  # logical values (0..1 for triggers, else -1..1)
        self._held_buttons: Set[Key] = set()
        # NEW: one-shot presses to apply next publish
        self._pending_one_shot_keys: Set[Key] = set()

        # Publisher
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        topic = f"{ocs_ns.strip('/')}/joy"
        self._pub = self.create_publisher(Joy, topic, qos)
        self.get_logger().info(f"Publishing Joy on '{topic}' at {self._joy_rate_hz} Hz")

        period = 1.0 / max(self._joy_rate_hz, 1.0)
        self._timer = self.create_timer(period, self._on_timer)

    # ---------- Public API ----------
    def press(self, plugin: str, function: str) -> None:
        # Record a one-shot press; mapping will occur in the timer publish.
        self._pending_one_shot_keys.add(Key(plugin, function))

    def hold(self, plugin: str, function: str) -> None:
        # Record a persistent press; mapping will occur in the timer publish.
        self._held_buttons.add(Key(plugin, function))

    def release(self, plugin: str, function: str) -> None:
        key = Key(plugin, function)
        self._held_buttons.discard(key)
        self._pending_one_shot_keys.discard(key)

    def deflect(self, plugin: str, function: str, value: float) -> None:
        # Store logical intent; mapping/validation occurs in timer publish.
        self._deflected_axes[Key(plugin, function)] = float(value)

    # ---------- Internals ----------
    def _on_timer(self) -> None:
        # Determine if a mode switch is required based on *current intents*
        intents: Set[Key] = set(self._held_buttons) | set(self._pending_one_shot_keys) | set(self._deflected_axes.keys())
        target_modes: Set[str] = set()

        for key in intents:
            m = self._find_mode_for_key(key)
            if m is None:
                raise KeyError(f"{key} not found in any loaded mode")
            if m != self._active_mode:
                target_modes.add(m)

        if len(target_modes) > 1:
            # Impossible configuration: inputs span multiple non-active modes
            raise RuntimeError(f"Conflicting intents across multiple modes: {sorted(target_modes)}")
        elif len(target_modes) == 1:
            target = next(iter(target_modes))
            # Publish a *switch-only* Joy: only the meta button for that config is pressed
            if target not in self._config_to_button:
                raise RuntimeError(f"No meta button mapped for target mode '{target}'")
            switch_btn = self._config_to_button[target]

            # First message: switch-only
            msg_switch = Joy()
            msg_switch.header.stamp = self.get_clock().now().to_msg()
            msg_switch.axes = self._neutral_axes()
            msg_switch.buttons = [0] * BUTTON_COUNT
            if 0 <= switch_btn < BUTTON_COUNT:
                msg_switch.buttons[switch_btn] = 1
            self._pub.publish(msg_switch)

            # Wait half the timer interval
            time.sleep(0.5 / max(self._joy_rate_hz, 1.0))

            # Assume manager switched; update local mode
            self._active_mode = target

        # Now publish the *actual* Joy based on current active mode and intents
        axes = self._neutral_axes()
        buttons = [0] * BUTTON_COUNT
        mode = self._modes[self._active_mode]

        # Apply held buttons
        for key in list(self._held_buttons):
            if key in mode.button_map:
                buttons[mode.button_map[key]] = 1
            else:
                # held key not in this mode -> impossible combo
                raise RuntimeError(f"Held button {key} not available in active mode '{self._active_mode}'")

        # Apply one-shot presses
        for key in list(self._pending_one_shot_keys):
            if key in mode.button_map:
                buttons[mode.button_map[key]] = 1
            else:
                raise RuntimeError(f"Pressed button {key} not available in active mode '{self._active_mode}'")
        # Clear one-shots after publishing
        self._pending_one_shot_keys.clear()

        # Apply axes (with logical->raw mapping, and validation by axis index)
        for key, logical in list(self._deflected_axes.items()):
            if key not in mode.axis_map:
                raise RuntimeError(f"Axis {key} not available in active mode '{self._active_mode}'")
            idx = mode.axis_map[key]
            # Validate ranges now that we know the axis index
            if idx in SPECIAL_TRIGGER_AXES:
                if not (0.0 <= logical <= 1.0):
                    raise ValueError(f"LT/RT logical value must be in [0, 1], got {logical}")
            else:
                if not (-1.0 <= logical <= 1.0):
                    raise ValueError(f"Axis logical value must be in [-1, 1], got {logical}")
            axes[idx] = self._logical_to_raw(idx, logical)

        # Publish final message
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes
        msg.buttons = buttons
        self._pub.publish(msg)

    def _set_button(self, plugin: str, function: str, value: int, persistent: bool) -> None:
        # UNUSED in the simplified flow but kept for API compatibility if called internally.
        key = Key(plugin, function)
        if value:
            if persistent:
                self._held_buttons.add(key)
            else:
                self._pending_one_shot_keys.add(key)
        else:
            self._held_buttons.discard(key)
            self._pending_one_shot_keys.discard(key)

    def _parse_mode(self, name: str, cfg: dict, reserved_buttons: Set[int]) -> Mode:
        buttons = {}
        axes = {}

        for k, v in (cfg.get("buttons") or {}).items():
            idx = int(k)
            if idx in reserved_buttons:
                continue
            plugin = (v or {}).get("plugin", "") or ""
            func = (v or {}).get("function", "") or ""
            if plugin and func:
                buttons[Key(plugin, func)] = idx

        for k, v in (cfg.get("axes") or {}).items():
            idx = int(k)
            plugin = (v or {}).get("plugin", "") or ""
            func = (v or {}).get("function", "") or ""
            if plugin and func:
                axes[Key(plugin, func)] = idx

        return Mode(name=name, button_map=buttons, axis_map=axes)

    def _get_mapping_for_key(self, key: Key) -> Mapping:
        mode = self._modes[self._active_mode]
        if key in mode.button_map:
            return Mapping(button_idx=mode.button_map[key], axis_idx=None)
        if key in mode.axis_map:
            return Mapping(button_idx=None, axis_idx=mode.axis_map[key])
        m = self._find_mode_for_key(key)
        if m is None:
            raise KeyError(f"{key} not found in any loaded mode")
        raise KeyError(f"{key} exists in mode '{m}' but not in active mode '{self._active_mode}'")

    def _key_available_in_mode(self, mode_name: str, key: Key) -> bool:
        mode = self._modes[mode_name]
        return key in mode.button_map or key in mode.axis_map

    def _find_mode_for_key(self, key: Key) -> Optional[str]:
        for name, mode in self._modes.items():
            if key in mode.button_map or key in mode.axis_map:
                return name
        return None

    def _attempt_mode_switch(self, target_mode: Optional[str], reason: str) -> None:
        # Kept for backward compatibility; not used in simplified switching.
        if target_mode is None or target_mode == self._active_mode:
            return
        # Minimal safety check (not invoked in new flow)
        missing: Set[Key] = set()
        for key in self._held_buttons:
            if not self._key_available_in_mode(target_mode, key):
                missing.add(key)
        for key in self._deflected_axes.keys():
            if not self._key_available_in_mode(target_mode, key):
                missing.add(key)
        if missing:
            details = ", ".join([f"{k.plugin}::{k.function}" for k in sorted(missing, key=lambda x: (x.plugin, x.function))])
            raise RuntimeError(
                f"Cannot switch mode to '{target_mode}' for {reason}: active inputs not present in target mode: {details}"
            )
        self._active_mode = target_mode

    # ----- Mapping & neutral helpers -----
    def _neutral_axes(self) -> list[float]:
        return list(AXIS_NEUTRALS)

    def _reset_physical_state(self) -> None:
        self._axes = self._neutral_axes()
        self._buttons = [0] * BUTTON_COUNT

    def _logical_to_raw(self, axis_idx: int, value: float) -> float:
        """
        Convert a logical axis value to raw Joy space.
        - LT/RT (2,5): logical [0..1] -> raw [1..-1] via raw = 1 - 2*value
        - others:      logical [-1..1] passthrough
        """
        if axis_idx in SPECIAL_TRIGGER_AXES:
            return 1.0 - 2.0 * value
        return value


def main():
    rclpy.init()
    node = FakeJoyPublisher()
    #node.deflect("hector_gamepad_manager_plugins::DrivePlugin", "drive", 0.5)
    #node.deflect("hector_gamepad_manager_plugins::DrivePlugin", "steer", 0.1)
    #node.deflect("hector_gamepad_manager_plugins::FlipperPlugin", "flipper_back_down", 1.0)
    #node.hold("hector_gamepad_manager_plugins::FlipperPlugin", "flipper_front_up")
    node.deflect("hector_gamepad_manager_plugins::ManipulationPlugin", "move_up_down", -0.5)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
