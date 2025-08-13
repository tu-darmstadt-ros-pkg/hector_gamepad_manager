#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Joy
import yaml
from ament_index_python.packages import get_package_share_directory

# =========================
# Axis layout (Joy message)
# =========================
# 0: LX     neutral 0
# 1: LY     neutral 0
# 2: LT     neutral +1   (special trigger)
# 3: RX     neutral 0
# 4: RY     neutral 0
# 5: RT     neutral +1   (special trigger)
# 6: DPad X neutral 0
# 7: DPad Y neutral 0
AXIS_COUNT = 8
BUTTON_COUNT = 11

AXIS_NEUTRALS = [0.0] * AXIS_COUNT
AXIS_NEUTRALS[2] = 1.0  # LT neutral
AXIS_NEUTRALS[5] = 1.0  # RT neutral

# Axes that behave like "buttons" logically: API uses [0,1], Joy raw must be [1..-1]
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

def _request_params_sync(node: Node, target_node_name: str, names: Tuple[str, ...]) -> Dict[str, Optional[str]]:
    client = node.create_client(GetParameters, f"{target_node_name}/get_parameters")
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"Parameter service not available: {target_node_name}/get_parameters")
    req = GetParameters.Request()
    req.names = list(names)
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    if not fut.done() or fut.result() is None:
        raise RuntimeError(f"Failed to get parameters from {target_node_name}")
    out = {}
    for n, v in zip(names, fut.result().values):
        out[n] = v.string_value
    return out


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

        manager_fqn = f"{self.get_namespace().rstrip('/')}/{manager_node_basename}".replace("//", "/")
        params = _request_params_sync(self, manager_fqn, ("config_name", "ocs_namespace"))
        config_name = params.get("config_name") or "athena"
        ocs_ns = params.get("ocs_namespace") or "ocs"
        self.get_logger().info(f"Using manager config_name='{config_name}', ocs_namespace='{ocs_ns}'")

        meta = _load_yaml(_pkg_config_path("hector_gamepad_manager", config_name))
        self._default_mode_name: str = meta.get("default_config", "driving")
        self._config_switch_reserved_buttons: Set[int] = set()
        modes_to_load: Dict[str, str] = {}

        for k, v in (meta.get("buttons") or {}).items():
            idx = int(k)
            pkg = (v or {}).get("package", "") or ""
            cfg = (v or {}).get("config", "") or ""
            if pkg and cfg:
                modes_to_load[cfg] = pkg
                self._config_switch_reserved_buttons.add(idx)

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

        # Physical Joy state (raw values) and logical intents
        self._axes = self._neutral_axes()
        self._buttons = [0] * BUTTON_COUNT

        # We store *logical* axis intents here (0..1 for LT/RT, -1..1 for others)
        self._deflected_axes: Dict[Key, float] = {}
        self._held_buttons: Set[Key] = set()

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
        self._set_button(plugin, function, value=1, persistent=False)

    def hold(self, plugin: str, function: str) -> None:
        self._set_button(plugin, function, value=1, persistent=True)

    def release(self, plugin: str, function: str) -> None:
        key = Key(plugin, function)
        mapping = self._get_mapping_for_key(key)
        if mapping.button_idx is None:
            raise ValueError(f"{key} is not a button in active mode '{self._active_mode}'")
        self._buttons[mapping.button_idx] = 0
        self._held_buttons.discard(key)

    def deflect(self, plugin: str, function: str, value: float) -> None:
        key = Key(plugin, function)
        # ensure mode
        if not self._key_available_in_mode(self._active_mode, key):
            target = self._find_mode_for_key(key)
            self._attempt_mode_switch(target, reason=f"deflect({key})")

        mapping = self._get_mapping_for_key(key)
        if mapping.axis_idx is None:
            raise ValueError(f"{key} is not an axis in active mode '{self._active_mode}'")

        # Validate by axis kind and convert logical -> raw
        if mapping.axis_idx in SPECIAL_TRIGGER_AXES:
            if not (0.0 <= value <= 1.0):
                raise ValueError("LT/RT logical value must be in [0, 1]")
        else:
            if not (-1.0 <= value <= 1.0):
                raise ValueError("Axis value must be in [-1, 1]")

        self._deflected_axes[key] = float(value)
        self._axes[mapping.axis_idx] = self._logical_to_raw(mapping.axis_idx, float(value))

    # ---------- Internals ----------
    def _on_timer(self) -> None:
        # Re-assert held/deflected intents to physical state before publishing
        for key in list(self._held_buttons):
            m = self._get_mapping_for_key(key)
            if m.button_idx is not None:
                self._buttons[m.button_idx] = 1
        for key, logical in list(self._deflected_axes.items()):
            m = self._get_mapping_for_key(key)
            if m.axis_idx is not None:
                self._axes[m.axis_idx] = self._logical_to_raw(m.axis_idx, logical)

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = list(self._axes)
        msg.buttons = list(self._buttons)
        self._pub.publish(msg)

    def _set_button(self, plugin: str, function: str, value: int, persistent: bool) -> None:
        key = Key(plugin, function)
        if not self._key_available_in_mode(self._active_mode, key):
            target = self._find_mode_for_key(key)
            self._attempt_mode_switch(target, reason=f"button({key})")
        mapping = self._get_mapping_for_key(key)
        if mapping.button_idx is None:
            raise ValueError(f"{key} is not a button in active mode '{self._active_mode}'")
        self._buttons[mapping.button_idx] = 1 if value else 0
        if persistent and value:
            self._held_buttons.add(key)
        else:
            self._held_buttons.discard(key)

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
        if target_mode is None:
            raise KeyError(f"No mode contains requested input ({reason})")
        if target_mode == self._active_mode:
            return

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

        self.get_logger().info(f"Switching mode: '{self._active_mode}' -> '{target_mode}' due to {reason}")
        self._reset_physical_state()
        self._active_mode = target_mode

        # Re-apply logical intents into physical state (with proper mapping)
        for key in list(self._held_buttons):
            mode = self._modes[self._active_mode]
            if key in mode.button_map:
                self._buttons[mode.button_map[key]] = 1
            else:
                self._held_buttons.discard(key)
        for key, logical in list(self._deflected_axes.items()):
            mode = self._modes[self._active_mode]
            if key in mode.axis_map:
                idx = mode.axis_map[key]
                self._axes[idx] = self._logical_to_raw(idx, logical)
            else:
                self._deflected_axes.pop(key, None)

    # ----- Mapping & neutral helpers -----
    def _neutral_axes(self) -> list[float]:
        return list(AXIS_NEUTRALS)

    def _reset_physical_state(self) -> None:
        self._axes = self._neutral_axes()
        self._buttons = [0] * BUTTON_COUNT

    def _logical_to_raw(self, axis_idx: int, value: float) -> float:
        """
        Convert a logical axis value to raw Joy space.
        - LT/RT (2,5): logical [0..1] -> raw [1..-1]  via  raw = 1 - 2*logical
        - others:      logical [-1..1] passthrough
        """
        if axis_idx in SPECIAL_TRIGGER_AXES:
            return 1.0 - 2.0 * value
        return value


# -----------------------------
# Entrypoint (optional run)
# -----------------------------
def main():
    rclpy.init()
    node = FakeJoyPublisher(
        manager_node_basename="/hector_gamepad_manager_node_aljoschalegionpro_984685_8407127964393618653"
    )
    node.deflect("hector_gamepad_manager_plugins::DrivePlugin", "drive", 0.5)
    node.deflect("hector_gamepad_manager_plugins::DrivePlugin", "steer", 0.1)
    node.deflect("hector_gamepad_manager_plugins::FlipperPlugin", "flipper_back_down", 1.0)
    node.hold("hector_gamepad_manager_plugins::FlipperPlugin", "flipper_front_up")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
