#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Locate action server - aligns base using detection pixel errors.
"""

import json
import time
import threading
from dataclasses import dataclass
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from winterhack_interfaces.action import Locate


@dataclass
class PIDState:
    """State for a simple PID controller."""
    integral: float = 0.0
    prev_error: float = 0.0
    prev_time: float | None = None


class LocateNode(Node):
    """Align base using detection pixel errors for yaw and forward motion."""

    def __init__(self):
        """Initialize parameters, subscriptions, publisher, and control state."""
        super().__init__("locate")
        self._cb_group = ReentrantCallbackGroup()

        # Topics
        self.declare_parameter("detections_topic", "/color_detection/detection_info")
        self.declare_parameter("cmd_vel_topic", "controller/cmd_vel")

        # Detection selection
        self.declare_parameter("color_priority", ["GREEN"])

        # Gates
        self.declare_parameter("detection_timeout_s", 0.5)

        # Control
        self.declare_parameter("k_yaw_px", 0.002)  # rad/s per pixel horizontal error (P fallback)
        self.declare_parameter("k_fwd_z", 0.8)     # m/s per pixel vertical error (P fallback)
        # Desired pixel location for the detected object (-1 uses frame center).
        self.declare_parameter("target_u_px", 320.0)
        self.declare_parameter("target_v_px", 320.0)
        self.declare_parameter("use_pid", True)
        self.declare_parameter("yaw_kp", 0.1)
        self.declare_parameter("yaw_ki", 0.02)
        self.declare_parameter("yaw_kd", 0.01)
        self.declare_parameter("fwd_kp", 0.1)
        self.declare_parameter("fwd_ki", 0.02)
        self.declare_parameter("fwd_kd", 0.01)
        self.declare_parameter("yaw_i_max", 0.02)
        self.declare_parameter("fwd_i_max", 0.02)
        self.declare_parameter("pid_dt_min", 0.02)
        self.declare_parameter("yaw_max", 0.2)
        self.declare_parameter("v_max", 0.2)
        self.declare_parameter("deadband_px", 6)
        self.declare_parameter("log_control", True)
        self.declare_parameter("log_every_n", 1)
        self.declare_parameter("alignment_timeout_s", 10.0)

        self.latest_detection: dict[str, Any] | None = None
        self.last_detection_time = None
        self.yaw_pid = PIDState()
        self.fwd_pid = PIDState()
        self.fwd_within_deadband_count = 0
        self.log_counter = 0
        self.in_deadband = False
        self._last_success = False
        self._last_message = ""
        self._timed_out = False
        self._active = False
        self.sub_det = None

        self.pub_cmd = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)

        self.timer = self.create_timer(0.05, self._loop, callback_group=self._cb_group)  # 20 Hz

    def reset_result(self) -> None:
        """Reset the last alignment result."""
        self._last_success = False
        self._last_message = ""
        self._timed_out = False

    def get_result(self) -> tuple[bool, str]:
        """Return the last alignment result (success, message)."""
        return self._last_success, self._last_message

    def start_alignment(self, *, reset_detection: bool = False) -> None:
        """Enable alignment control and clear prior status."""
        self.reset_result()
        self.in_deadband = False
        self.fwd_within_deadband_count = 0
        self._active = True
        if self.sub_det is None:
            self.sub_det = self.create_subscription(
                String,
                self.get_parameter("detections_topic").value,
                self._on_detection,
                10,
                callback_group=self._cb_group,
            )
        if reset_detection:
            self.latest_detection = None
            self.last_detection_time = None

    def stop_alignment(self) -> None:
        """Disable alignment control and stop the base."""
        self._active = False
        if self.sub_det is not None:
            self.destroy_subscription(self.sub_det)
            self.sub_det = None
        self.latest_detection = None
        self.last_detection_time = None
        self._publish_stop()

    def _publish_stop(self) -> None:
        """Publish zero velocity command."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)

    def _on_shutdown(self) -> None:
        """Ensure base is stopped on ROS shutdown."""
        self._publish_stop()

    # --- Callbacks ---
    def _on_detection(self, msg: String):
        """Parse detection JSON and cache the chosen detection."""
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        detections = payload.get("detections", [])
        if not detections:
            return

        chosen = self._select_detection(detections)
        self.latest_detection = chosen
        self.last_detection_time = self.get_clock().now()

    def _loop(self):
        """Run the alignment loop using the latest detection."""
        if not self._active:
            return
        if self.latest_detection is None or self.last_detection_time is None:
            return

        timeout_s = float(self.get_parameter("detection_timeout_s").value)
        if timeout_s > 0.0:
            age_ns = (self.get_clock().now() - self.last_detection_time).nanoseconds
            if age_ns > int(timeout_s * 1e9):
                self._publish_stop()
                self._last_success = False
                self._last_message = "Detection timeout"
                self._timed_out = True
                self.get_logger().info("Detection timeout; stopping base")
                return

        det = self.latest_detection
        try:
            u = float(det.get("center_x"))
            v = float(det.get("center_y"))
        except (TypeError, ValueError):
            return

        frame_cx = det.get("frame_center_x")
        if frame_cx is None:
            return
        target_u_px = float(self.get_parameter("target_u_px").value)
        desired_u_px = float(frame_cx) if target_u_px < 0.0 else target_u_px
        x_error_px = desired_u_px - u
        dead_px = int(self.get_parameter("deadband_px").value)
        within_yaw_deadband = abs(x_error_px) <= dead_px

        if within_yaw_deadband:
            wz = 0.0
            self.yaw_pid.integral = 0.0
            self.yaw_pid.prev_error = 0.0
        elif bool(self.get_parameter("use_pid").value):
            wz = self._pid_step(
                self.yaw_pid,
                x_error_px,
                float(self.get_parameter("yaw_kp").value),
                float(self.get_parameter("yaw_ki").value),
                float(self.get_parameter("yaw_kd").value),
                float(self.get_parameter("yaw_i_max").value),
            )
            yaw_max = float(self.get_parameter("yaw_max").value)
            wz = max(-yaw_max, min(yaw_max, wz))
        else:
            wz = self._pixel_yaw(x_error_px)
        vx = 0.0
        y_error_px = None

        frame_cy = det.get("frame_center_y")
        if frame_cy is None:
            return
        target_v_px = float(self.get_parameter("target_v_px").value)
        desired_v_px = float(frame_cy) if target_v_px < 0.0 else target_v_px
        y_error_px = desired_v_px - v
        within_y_deadband = abs(y_error_px) <= int(self.get_parameter("deadband_px").value)
        if within_y_deadband:
            self.fwd_within_deadband_count += 1
            vx = 0.0
            self.fwd_pid.integral = 0.0
            self.fwd_pid.prev_error = 0.0
        else:
            self.fwd_within_deadband_count = 0
            if bool(self.get_parameter("use_pid").value):
                vx = self._pid_step(
                    self.fwd_pid,
                    y_error_px,
                    float(self.get_parameter("fwd_kp").value),
                    float(self.get_parameter("fwd_ki").value),
                    float(self.get_parameter("fwd_kd").value),
                    float(self.get_parameter("fwd_i_max").value),
                )
            else:
                k_fwd_px = float(self.get_parameter("k_fwd_z").value)
                vx = k_fwd_px * y_error_px
            v_max = float(self.get_parameter("v_max").value)
            vx = max(-v_max, min(v_max, vx))

        # Stop publishing once both errors are within deadbands.
        if within_yaw_deadband and within_y_deadband:
            if not self.in_deadband:
                self._publish_stop()
                self.in_deadband = True
                self._last_success = True
                self._last_message = "Alignment complete"
                self.get_logger().info("Alignment complete")
            return
        self.in_deadband = False

        if bool(self.get_parameter("log_control").value):
            self.log_counter += 1
            log_every_n = int(self.get_parameter("log_every_n").value)
            if log_every_n <= 1 or self.log_counter % log_every_n == 0:
                y_err_val = float(y_error_px) if y_error_px is not None else float("nan")
                self.get_logger().info(
                    "ctrl x_error_px={:.1f} wz={:.3f} y_error_px={:.3f} vx={:.3f}".format(
                        x_error_px,
                        float(wz),
                        y_err_val,
                        float(vx),
                    )
                )

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        self.pub_cmd.publish(cmd)

    # --- Detection selection ---
    def _select_detection(self, detections: list[dict[str, Any]]) -> dict[str, Any]:
        """Pick a detection based on color priority or largest area."""
        color_priority = [
            c.upper()
            for c in self.get_parameter("color_priority").get_parameter_value().string_array_value
        ]
        chosen = None
        for color in color_priority:
            for det in detections:
                if str(det.get("color_name", "")).upper() == color:
                    chosen = det
                    break
            if chosen:
                break
        if not chosen:
            chosen = max(detections, key=lambda d: float(d.get("area", 0.0)))
        return chosen

    # --- Camera intrinsics ---
    def _pixel_yaw(self, x_error_px: float) -> float:
        """Compute yaw command from pixel error using P fallback."""
        dead_px = int(self.get_parameter("deadband_px").value)
        if abs(x_error_px) <= dead_px:
            return 0.0
        k_yaw_px = float(self.get_parameter("k_yaw_px").value)
        yaw_max = float(self.get_parameter("yaw_max").value)
        wz = k_yaw_px * x_error_px
        return max(-yaw_max, min(yaw_max, wz))

    def _pid_step(self, state: PIDState, error: float, kp: float, ki: float, kd: float, i_max: float) -> float:
        """Advance PID state and return control output."""
        now = self.get_clock().now()
        if state.prev_time is None:
            state.prev_time = now
            state.prev_error = error
            return kp * error

        dt = (now - state.prev_time).nanoseconds * 1e-9
        dt_min = float(self.get_parameter("pid_dt_min").value)
        if dt < dt_min:
            dt = dt_min

        state.integral += error * dt
        if i_max > 0.0:
            state.integral = max(-i_max, min(i_max, state.integral))

        derivative = (error - state.prev_error) / dt
        state.prev_error = error
        state.prev_time = now

        return kp * error + ki * state.integral + kd * derivative


class _AutoGoalHandle:
    def __init__(self, node: LocateNode):
        self._node = node

    def publish_feedback(self, _feedback):
        pass

    def abort(self):
        self._node.get_logger().warn("Auto-start locate aborted")

    def succeed(self):
        self._node.get_logger().info("Auto-start locate completed")


class LocateServer(LocateNode):
    """Action server for alignment using detection pixel errors."""

    def __init__(self):
        super().__init__()

        self.declare_parameter("auto_start", False)

        self._auto_running = False
        self._auto_start_timer = None
        self._cancel_requested = False

        self.action_server = ActionServer(
            self,
            Locate,
            "/locate",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info("Locate action server ready")

        if bool(self.get_parameter("auto_start").value):
            self._auto_start_timer = self.create_timer(0.1, self._auto_start_once)

    def goal_callback(self, _goal_request):
        if self._auto_running:
            self.get_logger().warn("Rejecting external goal while auto_start is running")
            return GoalResponse.REJECT
        self.get_logger().info("Received locate goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.get_logger().info("Locate goal cancelled")
        self._cancel_requested = True
        self.stop_alignment()
        return CancelResponse.ACCEPT

    def _auto_start_once(self):
        if self._auto_start_timer is not None:
            self._auto_start_timer.cancel()
            self.destroy_timer(self._auto_start_timer)
            self._auto_start_timer = None
        if self._auto_running:
            return
        self._auto_running = True
        thread = threading.Thread(target=self._run_auto_sequence, daemon=True)
        thread.start()

    def _run_auto_sequence(self):
        goal_handle = _AutoGoalHandle(self)
        try:
            self.execute_callback(goal_handle)
        finally:
            self._auto_running = False

    def execute_callback(self, goal_handle):
        feedback = Locate.Feedback()
        result = Locate.Result()

        self._cancel_requested = False
        self.start_alignment(reset_detection=False)

        feedback.stage = "Aligning"
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)

        timeout_s = float(self.get_parameter("alignment_timeout_s").value)
        start_time = time.monotonic()

        while rclpy.ok():
            if getattr(goal_handle, "is_cancel_requested", False) and goal_handle.is_cancel_requested:
                self.stop_alignment()
                result.success = False
                result.message = "Locate canceled"
                goal_handle.canceled()
                return result

            if self.in_deadband:
                self.stop_alignment()
                feedback.stage = "completed"
                feedback.progress = 1.0
                goal_handle.publish_feedback(feedback)
                result.success = True
                result.message = "Alignment complete"
                goal_handle.succeed()
                return result

            if self._timed_out:
                self.stop_alignment()
                result.success = False
                result.message = self._last_message or "Detection timeout"
                goal_handle.abort()
                return result

            if timeout_s > 0.0 and (time.monotonic() - start_time) > timeout_s:
                self.stop_alignment()
                result.success = False
                result.message = "Alignment timeout"
                goal_handle.abort()
                return result

            time.sleep(0.05)

        self.stop_alignment()
        result.success = False
        result.message = "ROS shutdown"
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    server = LocateServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
