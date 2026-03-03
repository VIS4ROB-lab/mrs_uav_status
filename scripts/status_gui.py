#!/usr/bin/env python3
"""
Lightweight Tkinter GUI replacement for the tmux-based mrs_uav_status interface.

The GUI subscribes to the same topics as the ncurses TUI (uav_status and
uav_status_short) and renders a compact dashboard per UAV. It is intentionally
minimal and dependency-free beyond Tkinter and rclpy so it can run inside the
existing ROS 2 environment without tmux.
"""
from __future__ import annotations

import argparse
import os
import threading
import time
from dataclasses import dataclass
from collections import deque
from typing import Dict, List, Optional, Tuple

import rclpy
from tkinter import messagebox
from geometry_msgs.msg import Pose, PoseStamped
from mrs_msgs.msg import Reference, UavStatus, UavStatusShort, ControlManagerDiagnostics
from mrs_msgs.srv import ReferenceStampedSrv, String as StringSrv
try:
    from mavros_msgs.srv import CommandTOL
except ImportError:
    CommandTOL = None
try:
    from mavros_msgs.msg import StatusText
except ImportError:
    StatusText = None
try:
    from mavros_msgs.msg import AttitudeTarget
except ImportError:
    AttitudeTarget = None
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import SetBool, Trigger
from tf2_geometry_msgs import do_transform_pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import MarkerArray
import tf2_ros

try:
    import tkinter as tk
    from tkinter import ttk
    from tkinter import scrolledtext
except ImportError as exc:  # pragma: no cover - Tk not present in headless builds
    raise RuntimeError(
        "Tkinter is required for status_gui.py. Install python3-tk or run with a display server."
    ) from exc


@dataclass
class UavSnapshot:
    """Container for the latest message pair for a single UAV."""

    status: Optional[UavStatus] = None
    status_short: Optional[UavStatusShort] = None
    control_manager_diag: Optional[ControlManagerDiagnostics] = None
    apm_thrust: Optional[float] = None
    last_update: float = 0.0


class StatusCollector(Node):
    """ROS 2 node that gathers status messages for one or more UAVs."""

    def __init__(self, uavs: List[str], status_topic: str, status_short_topic: str):
        super().__init__("mrs_uav_status_gui")
        self._lock = threading.Lock()
        self._use_apm_takeoff = os.environ.get("UAV_FCU", "").lower() == "apm"
        self._data: Dict[str, UavSnapshot] = {name: UavSnapshot() for name in uavs}
        self._svc_clients: Dict[str, Dict[str, object]] = {}
        self._turbo_enabled: Dict[str, bool] = {name: False for name in uavs}
        self._turbo_prev_constraint: Dict[str, Optional[str]] = {name: None for name in uavs}
        self._safety_area_scales: Dict[str, float] = {name: 2.0 for name in uavs}  # Default 2.0 m
        self._status_logs: Dict[str, deque[Tuple[float, int, str]]] = {name: deque(maxlen=400) for name in uavs}

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=False)

        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        for uav in uavs:
            self.create_subscription(
                UavStatus,
                f"/{uav}/{status_topic}",
                lambda msg, name=uav: self._handle_status(name, msg),
                qos,
            )
            self.create_subscription(
                UavStatusShort,
                f"/{uav}/{status_short_topic}",
                lambda msg, name=uav: self._handle_status_short(name, msg),
                qos,
            )
            self.create_subscription(
                ControlManagerDiagnostics,
                f"/{uav}/control_manager/diagnostics",
                lambda msg, name=uav: self._handle_control_manager_diag(name, msg),
                qos,
            )
            self.create_subscription(
                MarkerArray,
                f"/{uav}/safety_area_manager/static_markers",
                lambda msg, name=uav: self._handle_safety_area_markers(name, msg),
                qos,
            )
            if StatusText is not None:
                self.create_subscription(
                    StatusText,
                    f"/{uav}/mavros/statustext/recv",
                    lambda msg, name=uav: self._handle_status_text(name, msg),
                    qos,
                )
            if self._use_apm_takeoff and AttitudeTarget is not None:
                self.create_subscription(
                    AttitudeTarget,
                    f"/{uav}/mavros/setpoint_raw/target_attitude",
                    lambda msg, name=uav: self._handle_apm_target_attitude(name, msg),
                    qos,
                )

            self._svc_clients[uav] = {
                "ref": self.create_client(ReferenceStampedSrv, f"/{uav}/control_manager/reference"),
                "constraints": self.create_client(StringSrv, f"/{uav}/constraint_manager/set_constraints"),
                "hover": self.create_client(Trigger, f"/{uav}/control_manager/hover"),
                "arming": self.create_client(SetBool, f"/{uav}/hw_api/arming"),
                "offboard": self.create_client(Trigger, f"/{uav}/hw_api/offboard"),
                "takeoff": self.create_client(Trigger, f"/{uav}/uav_manager/takeoff"),
                "takeoff_apm": self.create_client(Trigger, f"/{uav}/uav_manager/takeoff_apm"),
                "land": self.create_client(Trigger, f"/{uav}/uav_manager/land"),
                "toggle_output": self.create_client(SetBool, f"/{uav}/control_manager/toggle_output"),
            }

    def _handle_status(self, uav: str, msg: UavStatus) -> None:
        with self._lock:
            snap = self._data[uav]
            snap.status = msg
            snap.last_update = self._now()

    def _handle_status_short(self, uav: str, msg: UavStatusShort) -> None:
        with self._lock:
            snap = self._data[uav]
            snap.status_short = msg
            snap.last_update = self._now()

    def _handle_control_manager_diag(self, uav: str, msg: ControlManagerDiagnostics) -> None:
        with self._lock:
            snap = self._data[uav]
            snap.control_manager_diag = msg
            snap.last_update = self._now()

    def _handle_safety_area_markers(self, uav: str, msg: MarkerArray) -> None:
        """Extract safety area bounds from marker array and compute optimal scale."""
        if not msg.markers:
            return
        
        # Extract x and y coordinates from all markers
        x_coords = []
        y_coords = []
        for marker in msg.markers:
            # Marker position
            x_coords.append(marker.pose.position.x)
            y_coords.append(marker.pose.position.y)
            # Also extract from scale/geometry if available
            # For LINE_STRIP or similar, points are in marker.points
            if hasattr(marker, 'points') and marker.points:
                for point in marker.points:
                    x_coords.append(point.x)
                    y_coords.append(point.y)
        
        if x_coords and y_coords:
            x_min, x_max = min(x_coords), max(x_coords)
            y_min, y_max = min(y_coords), max(y_coords)
            width = x_max - x_min
            height = y_max - y_min
            max_dim = max(width, height) if max(width, height) > 0 else 10.0
            # Scale as 1/10 of the safety area size, with minimum of 0.1m
            computed_scale = max(0.1, max_dim / 10.0)
            self._safety_area_scales[uav] = computed_scale

    def _handle_status_text(self, uav: str, msg: object) -> None:
        with self._lock:
            text_raw = getattr(msg, "text", "")
            text = text_raw.strip() if text_raw else ""
            if text:
                self._status_logs[uav].append((self._now(), int(getattr(msg, "severity", 6)), text))

    def pop_status_logs(self, name: str) -> List[Tuple[float, int, str]]:
        with self._lock:
            logs = list(self._status_logs.get(name, []))
            if name in self._status_logs:
                self._status_logs[name].clear()
            return logs

    def _handle_apm_target_attitude(self, uav: str, msg: object) -> None:
        with self._lock:
            snap = self._data[uav]
            snap.apm_thrust = float(getattr(msg, "thrust", 0.0))

    def get_snapshot(self) -> Dict[str, UavSnapshot]:
        with self._lock:
            return {
                name: UavSnapshot(
                    status=val.status,
                    status_short=val.status_short,
                    control_manager_diag=val.control_manager_diag,
                    apm_thrust=val.apm_thrust,
                    last_update=val.last_update,
                )
                for name, val in self._data.items()
            }

    def get_latest(self, name: str) -> Optional[UavSnapshot]:
        with self._lock:
            return self._data.get(name)

    def get_safety_area_scale(self, name: str) -> float:
        """Get the computed safety area scale for a UAV, or default if not available."""
        return self._safety_area_scales.get(name, 2.0)

    def is_apm_fcu(self) -> bool:
        return self._use_apm_takeoff

    def _get_cmd_state(self, snap: UavSnapshot) -> Tuple[float, float, float, float, str]:
        status = snap.status
        if status:
            return status.cmd_x, status.cmd_y, status.cmd_z, status.cmd_hdg, status.odom_frame
        return 0.0, 0.0, 0.0, 0.0, "map"

    def send_offset(self, name: str, dx: float, dy: float, dz: float, dh: float, global_mode: bool) -> None:
        snap = self.get_latest(name)
        if snap is None:
            self.get_logger().warning(f"No UAV named {name} registered.")
            return
        ref_client = self._svc_clients[name]["ref"]
        if not ref_client.service_is_ready():
            ref_client.wait_for_service(timeout_sec=0.2)

        cmd_x, cmd_y, cmd_z, cmd_hdg, odom_frame = self._get_cmd_state(snap)
        target_frame = odom_frame if global_mode else f"{name}/fcu_untilted"

        request = ReferenceStampedSrv.Request()

        if global_mode:
            request.reference.position.x = cmd_x + dx
            request.reference.position.y = cmd_y + dy
            request.reference.position.z = cmd_z + dz
            request.reference.heading = cmd_hdg + dh
            request.header.frame_id = odom_frame
        else:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = odom_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = cmd_x
            pose_stamped.pose.position.y = cmd_y
            pose_stamped.pose.position.z = cmd_z
            q = quaternion_from_euler(0.0, 0.0, cmd_hdg)
            pose_stamped.pose.orientation.x = q[0]
            pose_stamped.pose.orientation.y = q[1]
            pose_stamped.pose.orientation.z = q[2]
            pose_stamped.pose.orientation.w = q[3]

            try:
                transform = self._tf_buffer.lookup_transform(target_frame, odom_frame, rclpy.time.Time())
                pose_fcu: Pose = do_transform_pose(pose_stamped.pose, transform)
                yaw = euler_from_quaternion(
                    [
                        pose_fcu.orientation.x,
                        pose_fcu.orientation.y,
                        pose_fcu.orientation.z,
                        pose_fcu.orientation.w,
                    ]
                )[2]

                request.reference.position.x = pose_fcu.position.x + dx
                request.reference.position.y = pose_fcu.position.y + dy
                request.reference.position.z = pose_fcu.position.z + dz
                request.reference.heading = yaw + dh
                request.header.frame_id = target_frame
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f"Transform to {target_frame} failed ({exc}); falling back to global frame {odom_frame}."
                )
                request.reference.position.x = cmd_x + dx
                request.reference.position.y = cmd_y + dy
                request.reference.position.z = cmd_z + dz
                request.reference.heading = cmd_hdg + dh
                request.header.frame_id = odom_frame

        request.header.stamp = self.get_clock().now().to_msg()
        ref_client.call_async(request)

    def toggle_turbo(self, name: str, turbo_constraints: str, enable: bool) -> None:
        client = self._svc_clients[name]["constraints"]
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.2)

        request = StringSrv.Request()
        if enable:
            snap = self.get_latest(name)
            if snap and snap.status and snap.status.constraints:
                self._turbo_prev_constraint[name] = snap.status.constraints[0]
            request.value = turbo_constraints
        else:
            request.value = self._turbo_prev_constraint.get(name) or ""

        if request.value:
            client.call_async(request)
        else:
            self.get_logger().warning("No previous constraint to restore; skipping.")

        self._turbo_enabled[name] = enable

    def hover(self, name: str) -> None:
        client = self._svc_clients[name]["hover"]
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.2)
        client.call_async(Trigger.Request())

    def arm(self, name: str, arm: bool = True) -> None:
        client = self._svc_clients[name]["arming"]
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.2)
        request = SetBool.Request()
        request.data = arm
        client.call_async(request)

    def offboard(self, name: str) -> None:
        client = self._svc_clients[name]["offboard"]
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.2)
        client.call_async(Trigger.Request())

    def takeoff(self, name: str) -> None:
        if self._use_apm_takeoff:
            client = self._svc_clients[name]["takeoff_apm"]
            if not client.service_is_ready():
                client.wait_for_service(timeout_sec=0.2)
            client.call_async(Trigger.Request())
        else:
            client = self._svc_clients[name]["takeoff"]
            if not client.service_is_ready():
                client.wait_for_service(timeout_sec=0.2)
            client.call_async(Trigger.Request())

    def land(self, name: str) -> None:
        client = self._svc_clients[name]["land"]
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.2)
        client.call_async(Trigger.Request())

    def toggle_output(self, name: str) -> None:
        """Toggle control output ON/OFF based on current state (null_tracker status)."""
        snap = self.get_latest(name)
        client = self._svc_clients[name]["toggle_output"]
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.2)
        
        # Determine current state: if null_tracker is active, output is OFF
        # Toggle to opposite state
        request = SetBool.Request()
        if snap and snap.status:
            request.data = snap.status.null_tracker  # If null_tracker=True (OFF), set to True to turn ON
        else:
            request.data = True  # Default to enabling output if state unknown
        
        client.call_async(request)

    def _now(self) -> float:
        ros_time: Time = self.get_clock().now()
        return float(ros_time.nanoseconds) * 1e-9


class UavFrame(ttk.LabelFrame):
    """Visual block that renders one UAV."""

    def __init__(self, master: tk.Misc, uav_name: str):
        super().__init__(master, text=uav_name)
        self._rows: Dict[str, tk.Label] = {}

        labels = [
            ("last", "Last update"),
            ("diag", "Diagnostics"),
            ("mode", "Mode / RC"),
            ("armed", "Arming"),
            ("output", "Control Output"),
            ("position", "Position [m]", 42),
            ("setpoint", "Setpoint [m]", 42),
            ("controller", "Controller"),
            ("tracker", "Tracker"),
            ("constraint", "Constraint"),
            ("gain", "Gains"),
            ("odom", "Odom Hz"),
            ("battery", "Battery / Thrust"),
            ("health", "CPU / Temp / RAM"),
        ]

        for row_idx, (key, label, *width) in enumerate(labels):
            tk.Label(self, text=label, anchor="w", width=18).grid(row=row_idx, column=0, sticky="w", padx=(6, 4), pady=2)
            value = tk.Label(self, text="—", anchor="w", width=width[0] if width else 32)
            value.grid(row=row_idx, column=1, sticky="w", padx=(0, 6), pady=2)
            self._rows[key] = value

    def render(self, snapshot: UavSnapshot) -> None:
        age = time.time() - snapshot.last_update if snapshot.last_update else None
        age_text, age_color = self._age_text(age)
        self._set("last", age_text, age_color)

        status = snapshot.status
        short = snapshot.status_short
        diag = snapshot.control_manager_diag

        if status:
            self._render_full(status, snapshot.apm_thrust)
        elif short:
            self._render_short(short)
        else:
            self._set("diag", "waiting for data", "#b00020")
            return

        # Update output status from diagnostics if available
        if diag:
            output_status = "ENABLED" if diag.output_enabled else "DISABLED"
            self._set("output", output_status)

        if status:
            diag_text = self._diag_line(status)
            self._set("diag", diag_text)

    def _render_full(self, msg: UavStatus, apm_thrust: Optional[float] = None) -> None:
        mode = msg.hw_api_mode if msg.hw_api_mode else "—"
        rc_text = "RC" if msg.rc_mode else "autonomy"
        self._set("mode", f"{mode} ({rc_text})")

        armed = "ARMED" if msg.hw_api_armed else "DISARMED"
        ready = "ready" if msg.automatic_start_can_takeoff else "not ready"
        self._set("armed", f"{armed}, {ready}")

        # Control output status will be updated from diagnostics in render()
        # Default to ENABLED if diagnostics not yet available
        self._set("output", "ENABLED")

        self._set("position", self._format_pose(msg.odom_x, msg.odom_y, msg.odom_z, msg.odom_hdg))
        self._set("setpoint", self._format_pose(msg.cmd_x, msg.cmd_y, msg.cmd_z, msg.cmd_hdg))

        self._set("controller", self._first_with_options(msg.controllers))
        self._set("tracker", self._first_with_options(msg.trackers))
        self._set("constraint", self._first_with_options(msg.constraints))
        self._set("gain", self._first_with_options(msg.gains))

        self._set("odom", f"{msg.odom_hz:.1f} Hz in {msg.odom_frame}")

        battery = f"{msg.battery_volt:.1f} V, {msg.battery_curr:.1f} A"
        thrust_value = apm_thrust if apm_thrust is not None else msg.thrust
        thrust = f"thrust {thrust_value:.2f}"
        self._set("battery", f"{battery}; {thrust}")

        cpu = f"CPU {msg.cpu_load:.1f}%"
        temp = f"{msg.cpu_temperature:.1f} C" if msg.cpu_temperature > 0 else "temp n/a"
        if msg.total_ram > 0.0:
            used_pct = max(0.0, min(100.0, (1.0 - (msg.free_ram / msg.total_ram)) * 100.0))
            ram = f"RAM {used_pct:.1f}%"
        else:
            ram = ""
        self._set("health", f"{cpu}; {temp}; {ram}")

    def _render_short(self, msg: UavStatusShort) -> None:
        self._set("mode", "short status only")
        self._set("position", self._format_pose(msg.odom_x, msg.odom_y, msg.odom_z, msg.odom_hdg))
        self._set("setpoint", self._format_pose(msg.cmd_x, msg.cmd_y, msg.cmd_z, msg.cmd_hdg))
        self._set("diag", f"odom {msg.odom_hz:.1f} Hz")

    @staticmethod
    def _format_pose(x: float, y: float, z: float, hdg: float) -> str:
        return f"x={x:.2f}, y={y:.2f}, z={z:.2f}, hdg={hdg:.2f}"

    @staticmethod
    def _diag_line(msg: UavStatus) -> str:
        cm_state, _ = UavFrame._color_status(msg.control_manager_diag_color)
        hw_state, _ = UavFrame._color_status(msg.hw_api_color)
        return f"CM {cm_state} @ {msg.control_manager_diag_hz:.1f} Hz | HW {hw_state} @ {msg.hw_api_hz:.1f} Hz"

    @staticmethod
    def _color_status(code: int) -> Tuple[str, str]:
        """Map MRS color codes (GREEN=102, YELLOW=104, RED=103) to status strings."""
        if code == 102:  # GREEN
            return "OK", "#2e8540"
        if code == 104:  # YELLOW
            return "WARN", "#c17d0d"
        if code == 103:  # RED
            return "ERROR", "#b00020"
        return "UNKNOWN", "#808080"

    @staticmethod
    def _first_with_options(values: List[str]) -> str:
        if not values:
            return "—"
        if len(values) == 1:
            return values[0]
        return f"{values[0]} ({', '.join(values[1:])})"

    def _set(self, key: str, text: str, fg: Optional[str] = None) -> None:
        label = self._rows[key]
        label.configure(text=text)
        if fg:
            label.configure(fg=fg)

    @staticmethod
    def _age_text(age: Optional[float]) -> Tuple[str, Optional[str]]:
        if age is None:
            return "waiting for data", "#b00020"
        if age < 1.5:
            return "fresh", "#2e8540"
        if age < 3.5:
            return f"{age:.1f} s", "#c17d0d"
        return f"{age:.1f} s", "#b00020"


class RemotePanel(ttk.LabelFrame):
    """Buttons and key bindings matching the tmux remote handler keys."""

    def __init__(self, master: tk.Misc, collector: StatusCollector, uav_name: str, turbo_constraints: str, remote_scale: float):
        super().__init__(master, text=f"{uav_name} remote")
        self.collector = collector
        self.uav = uav_name
        self.turbo_constraints = turbo_constraints
        self.default_remote_scale = remote_scale  # Fallback if safety area not available

        self.global_mode = tk.BooleanVar(value=False)
        self.turbo_mode = tk.BooleanVar(value=False)

        row = 0
        output_state = "disabled" if self.collector.is_apm_fcu() else "normal"
        ttk.Button(self, text="Activate", command=self._toggle_output, state=output_state).grid(row=row, column=0, sticky="ew", padx=4, pady=2)
        offboard_state = "disabled" if self.collector.is_apm_fcu() else "normal"
        ttk.Button(self, text="Offboard", command=self._offboard, state=offboard_state).grid(
            row=row, column=1, sticky="ew", padx=4, pady=2
        )
        ttk.Button(self, text="Arm", command=self._arm).grid(row=row, column=2, sticky="ew", padx=4, pady=2)

        row += 1
        ttk.Button(self, text="Takeoff", command=self._takeoff).grid(row=row, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(self, text="Hover", command=self._hover).grid(row=row, column=1, sticky="ew", padx=4, pady=2)
        ttk.Button(self, text="Land", command=self._land).grid(row=row, column=2, sticky="ew", padx=4, pady=2)

        row += 1
        ttk.Button(self, text="a/h/Roll+ (left)", width=12, command=lambda: self._send_scaled(0.0, 1.0, 0.0, 0.0)).grid(row=row, column=0, padx=2, pady=2)
        ttk.Button(self, text="d/l/Roll- (right)", width=12, command=lambda: self._send_scaled(0.0, -1.0, 0.0, 0.0)).grid(row=row, column=2, padx=2, pady=2)
        ttk.Button(self, text="w/k/Pitch+ (forward)", width=12, command=lambda: self._send_scaled(1.0, 0.0, 0.0, 0.0)).grid(row=row, column=3, padx=2, pady=2)
        ttk.Button(self, text="s/j/Pitch- (back)", width=12, command=lambda: self._send_scaled(-1.0, 0.0, 0.0, 0.0)).grid(row=row, column=1, padx=2, pady=2)

        row += 1
        ttk.Button(self, text="r (thrust+)", width=12, command=lambda: self._send_scaled(0.0, 0.0, 1.0, 0.0)).grid(row=row, column=0, padx=2, pady=2)
        ttk.Button(self, text="f (thrust-)", width=12, command=lambda: self._send_scaled(0.0, 0.0, -1.0, 0.0)).grid(row=row, column=1, padx=2, pady=2)
        ttk.Button(self, text="q (yaw+)", width=12, command=lambda: self._send(0.0, 0.0, 0.0, 0.25)).grid(row=row, column=2, padx=2, pady=2)
        ttk.Button(self, text="e (yaw-)", width=12, command=lambda: self._send(0.0, 0.0, 0.0, -0.25)).grid(row=row, column=3, padx=2, pady=2)

    def _get_current_scale(self) -> float:
        """Get the current safety area scale from collector, fallback to default."""
        return self.collector.get_safety_area_scale(self.uav)

    def _send_scaled(self, dx_factor: float, dy_factor: float, dz_factor: float, dh: float) -> None:
        """Send offset with scale applied from safety area size."""
        scale = min(self._get_current_scale(), 2)
        self._send(dx_factor * scale, dy_factor * scale, dz_factor * scale, dh)

    def handle_key(self, keysym: str) -> bool:
        """Return True if the key was handled, mirroring tmux key map."""
        scale = min(self._get_current_scale(), 2)
        mapping = {
            ("w", "k", "Up"): (scale, 0.0, 0.0, 0.0),
            ("s", "j", "Down"): (-scale, 0.0, 0.0, 0.0),
            ("a", "h", "Left"): (0.0, scale, 0.0, 0.0),
            ("d", "l", "Right"): (0.0, -scale, 0.0, 0.0),
            ("r",): (0.0, 0.0, scale, 0.0),
            ("f",): (0.0, 0.0, -scale, 0.0),
            ("q",): (0.0, 0.0, 0.0, 0.25),
            ("e",): (0.0, 0.0, 0.0, -0.25),
        }

        for keys, offsets in mapping.items():
            if keysym in keys:
                self._send(*offsets)
                return True
        return False

    def _send(self, dx: float, dy: float, dz: float, dh: float) -> None:
        scale = 2.5 if self.turbo_mode.get() else 1.0
        self.collector.send_offset(self.uav, dx * scale, dy * scale, dz * scale, dh * scale, self.global_mode.get())

    def _toggle_turbo(self) -> None:
        self.collector.toggle_turbo(self.uav, self.turbo_constraints, self.turbo_mode.get())

    def _toggle_global(self) -> None:
        # No service call needed; the toggle is local state controlling send frame.
        pass

    def _hover(self) -> None:
        self.collector.hover(self.uav)

    def _arm(self) -> None:
        snap = self.collector.get_latest(self.uav)
        if snap is None or snap.status is None:
            messagebox.showwarning("No Data", f"No status data available for {self.uav}")
            return
        
        is_armed = snap.status.hw_api_armed
        action = "DISARM" if is_armed else "ARM"
        
        if messagebox.askyesno(f"Confirm {action}", f"Are you sure you want to {action} the UAV?"):
            self.collector.arm(self.uav, arm=not is_armed)

    def _offboard(self) -> None:
        self.collector.offboard(self.uav)

    def _takeoff(self) -> None:
        self.collector.takeoff(self.uav)

    def _land(self) -> None:
        self.collector.land(self.uav)

    def _toggle_output(self) -> None:
        self.collector.toggle_output(self.uav)


class ConsolePanel(ttk.LabelFrame):
    """Scrollable status text console for MAVROS StatusText messages."""

    _SEVERITY_STYLES: Dict[int, Tuple[str, str]] = {
        0: ("EMERGENCY", "#ff0033"),
        1: ("ALERT", "#ff3355"),
        2: ("CRITICAL", "#ff5533"),
        3: ("ERROR", "#ff7f00"),
        4: ("WARNING", "#d49f00"),
        5: ("NOTICE", "#248f24"),
        6: ("INFO", "#1b6ec2"),
        7: ("DEBUG", "#808080"),
    }

    def __init__(self, master: tk.Misc, uav_name: str):
        super().__init__(master, text=f"{uav_name} console")
        self._text = scrolledtext.ScrolledText(self, height=8, width=56, state="disabled", wrap="word")
        self._text.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        for severity, (_, color) in self._SEVERITY_STYLES.items():
            self._text.tag_configure(f"sev_{severity}", foreground=color)
        self._text.tag_configure("timestamp", foreground="#666666")

    def append(self, messages: List[Tuple[float, int, str]]) -> None:
        if not messages:
            return
        self._text.configure(state="normal")
        for stamp, severity, text in messages:
            label, _ = self._SEVERITY_STYLES.get(severity, (f"SEV{severity}", "#000000"))
            wall = time.strftime("%H:%M:%S", time.localtime(stamp))
            self._text.insert("end", f"[{wall}] ", ("timestamp",))
            self._text.insert("end", f"[{label}] ", (f"sev_{severity}",))
            self._text.insert("end", f"{text}\n")
        self._text.see("end")
        self._text.configure(state="disabled")

def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Python GUI replacement for the tmux status TUI.", add_help=True)
    parser.add_argument("--uavs", nargs="+", default=None, help="UAV names to watch (defaults to $UAV_NAME or uav1)")
    parser.add_argument("--status-topic", default="uav_status_acquisition/uav_status", help="Relative topic for UavStatus messages")
    parser.add_argument(
        "--status-short-topic", default="uav_status_acquisition/uav_status_short", help="Relative topic for UavStatusShort messages"
    )
    parser.add_argument("--turbo-constraints", default="fast", help="Constraint preset used when Turbo is toggled on")
    parser.add_argument("--remote-scale", type=float, default=2.0, help="Scale factor for remote control offsets (meters per step)")
    parser.add_argument("--refresh-ms", type=int, default=400, help="GUI refresh period in milliseconds")
    parser.add_argument("--title", default=None, help="Optional custom window title")
    args, ros_args = parser.parse_known_args(argv)

    if args.uavs is None:
        env_uav = os.environ.get("UAV_NAME", "uav1")
        args.uavs = [env_uav]

    args.ros_args = ros_args
    return args


def main(argv: Optional[List[str]] = None) -> None:
    args = parse_args(argv)

    rclpy.init(args=args.ros_args)

    collector = StatusCollector(args.uavs, args.status_topic, args.status_short_topic)
    executor = MultiThreadedExecutor()
    executor.add_node(collector)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title(args.title or "MRS UAV Status")

    frames = {}
    remote_panels: Dict[str, RemotePanel] = {}
    console_panels: Dict[str, ConsolePanel] = {}
    for idx, name in enumerate(args.uavs):
        container = ttk.Frame(root)
        container.grid(row=idx, column=0, padx=8, pady=6, sticky="nsew")
        status_frame = UavFrame(container, name)
        status_frame.grid(row=0, column=0, sticky="nsew")
        remote_frame = RemotePanel(container, collector, name, args.turbo_constraints, args.remote_scale)
        remote_frame.grid(row=1, column=0, sticky="nsew", pady=(4, 0))
        console_frame = ConsolePanel(container, name)
        console_frame.grid(row=2, column=0, sticky="nsew", pady=(4, 0))
        frames[name] = status_frame
        remote_panels[name] = remote_frame
        console_panels[name] = console_frame

    def refresh_ui() -> None:
        snapshot = collector.get_snapshot()
        for name, frame in frames.items():
            frame.render(snapshot.get(name, UavSnapshot()))
            logs = collector.pop_status_logs(name)
            if logs:
                console_panels[name].append(logs)
        root.after(args.refresh_ms, refresh_ui)

    # Bind keyboard to the first UAV's remote panel to mirror tmux controls.
    if remote_panels:
        default_uav = args.uavs[0]

        def on_key(event: tk.Event) -> None:  # type: ignore[name-defined]
            panel = remote_panels.get(default_uav)
            if panel and panel.handle_key(event.keysym):
                return

        for key in [
            "w",
            "k",
            "Up",
            "s",
            "j",
            "Down",
            "a",
            "h",
            "Left",
            "d",
            "l",
            "Right",
            "r",
            "f",
            "q",
            "e",
            "T",
            "G",
        ]:
            root.bind(key, on_key)

    def shutdown() -> None:
        root.destroy()
        executor.shutdown()
        collector.destroy_node()
        rclpy.shutdown()

    root.protocol("WM_DELETE_WINDOW", shutdown)
    root.after(args.refresh_ms, refresh_ui)
    try:
        root.mainloop()
    finally:
        if rclpy.ok():
            shutdown()


if __name__ == "__main__":
    main()
