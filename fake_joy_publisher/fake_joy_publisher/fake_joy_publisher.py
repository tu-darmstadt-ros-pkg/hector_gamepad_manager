#!/usr/bin/env python3
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Joy
import yaml
from ament_index_python.packages import get_package_share_directory


# -----------------------------
# Data structures
# -----------------------------
@dataclass(frozen=True)
class Key:
    plugin: str
    function: str


@dataclass
class Mapping:
    # exactly one of (button_idx, axis_idx) is not None
    button_idx: Optional[int] = None
    axis_idx: Optional[int] = None


@dataclass
class Mode:
    name: str
    button_map: Dict[Key, int]  # Key -> button index (0..10)
    axis_map: Dict[Key, int]  # Key -> axis index (0..7)


# -----------------------------
# Helpers
# -----------------------------
def _pkg_config_path(pkg: str, name: str) -> str:
    """Mimic manager's getPath(): <share>/<pkg>/config/<name>[.yaml]"""
    base = get_package_share_directory(pkg)
    filename = name if name.endswith(".yaml") else f"{name}.yaml"
    return f"{base}/config/{filename}"


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _request_params_sync(
    node: Node, target_node_name: str, names: Tuple[str, ...]
) -> Dict[str, Optional[str]]:
    """
    Get string parameters from another node (sync). Missing -> None.
    """
    srv = f"{target_node_name}/get_parameters"
    client = node.create_client(GetParameters, srv)
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"Parameter service not available: {srv}")

    req = GetParameters.Request()
    req.names = list(names)
    future = client.call_async(req)

    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"Failed to get parameters from {target_node_name}")

    result = {}
    for name, val in zip(names, future.result().values):
        # We only expect strings here
        result[name] = val.string_value
    return result


# -----------------------------
# Core Node / Library
# -----------------------------
class FakeJoyPublisher(Node):
    """
    Emulates a gamepad by publishing Joy messages and auto-switching the active
    configuration (mode) based on the hector_gamepad_manager meta-config.

    Public API:
      - press(plugin, function)
      - release(plugin, function)
      - hold(plugin, function)
      - deflect(plugin, function, value)    # for axes in [-1,1] (triggers/DPAD included)

    Behavior:
      * If you act on a (plugin,function) not present in the active mode, the node
        switches to the mode that contains it — unless that would conflict with
        currently held inputs; in that case a RuntimeError is raised.
      * Buttons reserved by the meta-config (used for config switching) are blocked
        in *all* modes.
      * Publishes Joy at a fixed frequency (param: joy_rate_hz).
    """

    def __init__(
        self,
        node_name: str = "fake_joy_publisher",
        manager_node_basename: str = "hector_gamepad_manager",
        joy_rate_hz: float = 50.0,
    ):
        super().__init__(node_name)

        # --- Parameters (this node) ---
        self.declare_parameter("joy_rate_hz", joy_rate_hz)
        self._joy_rate_hz = float(self.get_parameter("joy_rate_hz").value)

        # --- Discover manager params in the *same namespace* ---
        manager_fqn = (
            f"{self.get_namespace().rstrip('/')}/{manager_node_basename}".replace(
                "//", "/"
            )
        )
        params = _request_params_sync(
            self, manager_fqn, ("config_name", "ocs_namespace")
        )
        config_name = params.get("config_name") or "athena"
        ocs_ns = params.get("ocs_namespace") or "ocs"
        self.get_logger().info(
            f"Using manager config_name='{config_name}', ocs_namespace='{ocs_ns}'"
        )

        # --- Load meta-config ---
        meta = _load_yaml(_pkg_config_path("hector_gamepad_manager", config_name))
        self._default_mode_name: str = meta.get("default_config", "driving")
        self._config_switch_reserved_buttons: Set[int] = set()
        modes_to_load: Dict[str, str] = {}  # config_name -> pkg

        for k, v in (meta.get("buttons") or {}).items():
            idx = int(k)
            pkg = (v or {}).get("package", "") or ""
            cfg = (v or {}).get("config", "") or ""
            if pkg and cfg:
                modes_to_load[cfg] = pkg
                self._config_switch_reserved_buttons.add(idx)

        # --- Load all referenced per-mode configs ---
        self._modes: Dict[str, Mode] = {}
        for mode_name, pkg in modes_to_load.items():
            cfg = _load_yaml(_pkg_config_path(pkg, mode_name))
            self._modes[mode_name] = self._parse_mode(
                mode_name, cfg, self._config_switch_reserved_buttons
            )

        # Also ensure default mode is available (if not referenced by meta buttons)
        if self._default_mode_name not in self._modes:
            # assume default lives in hector_gamepad_manager unless otherwise provided
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

        # --- Active mode & state ---
        self._active_mode: str = self._default_mode_name

        # low-level Joy state (physical layout expected by the manager):
        # axes: [LX, LY, LT(raw 1..-1), RX, RY, RT(raw 1..-1), DPAD_X, DPAD_Y]
        # buttons: [A, B, X, Y, LB, RB, Back, Start, Guide, L3, R3]
        self._axes = [0.0] * 8
        self._buttons = [0] * 11

        # Track logical holds/deflections
        self._held_buttons: Set[Key] = set()
        self._deflected_axes: Dict[Key, float] = {}

        # Publisher
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        topic = f"{ocs_ns.strip('/')}/joy"
        self._pub = self.create_publisher(Joy, topic, qos)
        self.get_logger().info(f"Publishing Joy on '{topic}' at {self._joy_rate_hz} Hz")

        # Timer
        period = 1.0 / max(self._joy_rate_hz, 1.0)
        self._timer = self.create_timer(period, self._on_timer)

    # ------------- Public API -------------
    def press(self, plugin: str, function: str) -> None:
        self._set_button(plugin, function, value=1, persistent=False)

    def hold(self, plugin: str, function: str) -> None:
        self._set_button(plugin, function, value=1, persistent=True)

    def release(self, plugin: str, function: str) -> None:
        key = Key(plugin, function)
        mapping = self._get_mapping_for_key(key)
        if mapping.button_idx is None:
            raise ValueError(
                f"{key} is not a button in active mode '{self._active_mode}'"
            )
        self._buttons[mapping.button_idx] = 0
        self._held_buttons.discard(key)

    def deflect(self, plugin: str, function: str, value: float) -> None:
        if value < -1.0 or value > 1.0:
            raise ValueError("Axis value must be in [-1, 1]")
        key = Key(plugin, function)
        # if key not in active mode, try switching
        if not self._key_available_in_mode(self._active_mode, key):
            target = self._find_mode_for_key(key)
            self._attempt_mode_switch(target, reason=f"deflect({key})")
        mapping = self._get_mapping_for_key(key)
        if mapping.axis_idx is None:
            raise ValueError(
                f"{key} is not an axis in active mode '{self._active_mode}'"
            )

        # Set the raw axis (manager will transform LT/RT internally)
        self._axes[mapping.axis_idx] = float(value)
        self._deflected_axes[key] = float(value)

    # ------------- Internals -------------
    def _on_timer(self) -> None:
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = list(self._axes)
        msg.buttons = list(self._buttons)
        self._pub.publish(msg)

        # Re-assert held buttons & deflected axes so external resets don't drop them
        for key in list(self._held_buttons):
            m = self._get_mapping_for_key(key)
            if m.button_idx is not None:
                self._buttons[m.button_idx] = 1
        for key, val in list(self._deflected_axes.items()):
            m = self._get_mapping_for_key(key)
            if m.axis_idx is not None:
                self._axes[m.axis_idx] = val

    def _set_button(
        self, plugin: str, function: str, value: int, persistent: bool
    ) -> None:
        key = Key(plugin, function)
        # If (plugin,function) isn't available in the active mode, try to switch
        if not self._key_available_in_mode(self._active_mode, key):
            target = self._find_mode_for_key(key)
            self._attempt_mode_switch(target, reason=f"button({key})")

        mapping = self._get_mapping_for_key(key)
        if mapping.button_idx is None:
            raise ValueError(
                f"{key} is not a button in active mode '{self._active_mode}'"
            )

        # Guard: reserved buttons are blocked globally already when parsing configs
        self._buttons[mapping.button_idx] = 1 if value else 0
        if persistent and value:
            self._held_buttons.add(key)
        else:
            self._held_buttons.discard(key)

    # -------- Mode / Mapping logic --------
    def _parse_mode(self, name: str, cfg: dict, reserved_buttons: Set[int]) -> Mode:
        buttons = {}
        axes = {}

        # Buttons
        for k, v in (cfg.get("buttons") or {}).items():
            idx = int(k)
            if idx in reserved_buttons:
                # globally blocked for switching
                continue
            plugin = (v or {}).get("plugin", "") or ""
            func = (v or {}).get("function", "") or ""
            if plugin and func:
                buttons[Key(plugin, func)] = idx

        # Axes
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
        # not in active mode—try elsewhere to give a better error
        m = self._find_mode_for_key(key)
        if m is None:
            raise KeyError(f"{key} not found in any loaded mode")
        raise KeyError(
            f"{key} exists in mode '{m}' but not in active mode '{self._active_mode}'"
        )

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
        # Conflict detection: if we have any currently held/deflected keys that
        # do NOT exist in the target mode, switching would orphan those inputs.
        missing: Set[Key] = set()
        for key in self._held_buttons:
            if not self._key_available_in_mode(target_mode, key):
                missing.add(key)
        for key in self._deflected_axes.keys():
            if not self._key_available_in_mode(target_mode, key):
                missing.add(key)

        if missing:
            details = ", ".join(
                [
                    f"{k.plugin}::{k.function}"
                    for k in sorted(missing, key=lambda x: (x.plugin, x.function))
                ]
            )
            raise RuntimeError(
                f"Cannot switch mode to '{target_mode}' for {reason}: active inputs not present in target mode: {details}"
            )

        self.get_logger().info(
            f"Switching mode: '{self._active_mode}' -> '{target_mode}' due to {reason}"
        )
        # Clear physical state; logical holds/deflections are kept and reapplied if present in target.
        self._axes = [0.0] * 8
        self._buttons = [0] * 11
        self._active_mode = target_mode

        # Re-apply persistent intents that exist in target
        for key in list(self._held_buttons):
            mode = self._modes[self._active_mode]
            if key in mode.button_map:
                self._buttons[mode.button_map[key]] = 1
            else:
                # Drop holds that don't exist here
                self._held_buttons.discard(key)
        for key, val in list(self._deflected_axes.items()):
            mode = self._modes[self._active_mode]
            if key in mode.axis_map:
                self._axes[mode.axis_map[key]] = val
            else:
                self._deflected_axes.pop(key, None)


# -----------------------------
# Entrypoint (optional run)
# -----------------------------
def main():
    rclpy.init()
    node = FakeJoyPublisher(
        manager_node_basename="/hector_gamepad_manager_node_aljoschalegionpro_984685_8407127964393618653"
    )
    node.deflect("hector_gamepad_manager_plugins::DrivePlugin", "drive", 0.5)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
