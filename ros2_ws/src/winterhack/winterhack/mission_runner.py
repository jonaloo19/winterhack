#!/usr/bin/env python3
"""
Mission runner node coordinating Nav2 patrol and pick/drop actions.
"""

# README
# Required nodes: Nav2 bringup, detector node, pick server, drop server.
# Run: ros2 run winterhack mission_runner

import json
import math
import time
from enum import Enum
from typing import Callable, Iterable, Optional, List, Set, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from winterhack_interfaces.action import Drop, Locate, Pick

try:
    from tf_transformations import quaternion_from_euler  # type: ignore[import-not-found]
except Exception:
    quaternion_from_euler: Optional[Callable[[float, float, float], Tuple[float, float, float, float]]] = None


DEFAULT_GLOBAL_TARGETS = [(-1.4, -1.4), (1.4, 1.4), (-1.4, 1.4), (1.4, -1.4)]
DEFAULT_HOME = (0.0, 0.0)


class MissionPhase(Enum):
    INIT = "INIT"
    PATROL_SEARCH = "PATROL_SEARCH"
    STOP_SEARCH = "STOP_SEARCH"
    LOCATE = "LOCATE"
    PICK = "PICK"
    RETRIEVE_HOME = "RETRIEVE_HOME"
    DROP = "DROP"
    IDLE = "IDLE"


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw (rad) to quaternion."""
    if quaternion_from_euler is not None:
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    else:
        half = yaw * 0.5
        qx = 0.0
        qy = 0.0
        qz = math.sin(half)
        qw = math.cos(half)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def make_pose_stamped(
    x: float, y: float, yaw: float, frame_id: str, stamp=None
) -> PoseStamped:
    """Create a PoseStamped in the given frame."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp if stamp is not None else rclpy.time.Time().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation = yaw_to_quat(yaw)
    return pose


class MissionRunner(Node):
    """Phase-driven mission controller for patrol and pick/drop."""

    def __init__(self) -> None:
        super().__init__("mission_runner")

        self._cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("home_xy", [0.0, 0.0])
        self.declare_parameter("target_pick_count", 2)
        self.declare_parameter("nav_follow_waypoints_name", "/follow_waypoints")
        self.declare_parameter("nav_to_pose_name", "/navigate_to_pose")
        self.declare_parameter("locate_action_name", "/locate")
        self.declare_parameter("pick_action_name", "/pick")
        self.declare_parameter("drop_action_name", "/drop")
        self.declare_parameter("enable_locate", True)
        self.declare_parameter("enable_pick", True)
        self.declare_parameter("enable_drop", True)
        self.declare_parameter("enable_go_home", True)
        self.declare_parameter("detection_topic", "/color_detection/detection_info")
        self.declare_parameter("map_frame", "map")

        self._global_targets = list(DEFAULT_GLOBAL_TARGETS)
        self._home_xy = self._parse_xy(self.get_parameter("home_xy").value, DEFAULT_HOME)
        self._target_pick_count = int(self.get_parameter("target_pick_count").value)
        self._nav_follow_waypoints_name = str(
            self.get_parameter("nav_follow_waypoints_name").value
        )
        self._nav_to_pose_name = str(self.get_parameter("nav_to_pose_name").value)
        self._locate_action_name = str(self.get_parameter("locate_action_name").value)
        self._pick_action_name = str(self.get_parameter("pick_action_name").value)
        self._drop_action_name = str(self.get_parameter("drop_action_name").value)
        self._enable_locate = bool(self.get_parameter("enable_locate").value)
        self._enable_pick = bool(self.get_parameter("enable_pick").value)
        self._enable_drop = bool(self.get_parameter("enable_drop").value)
        self._enable_go_home = bool(self.get_parameter("enable_go_home").value)
        self._detection_topic = str(self.get_parameter("detection_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)

        self.detected_colour: Optional[str] = None
        self.detected_colours: Set[str] = set()
        self.picked_total = 0
        self._state = MissionPhase.INIT

        self._follow_client = ActionClient(
            self,
            FollowWaypoints,
            self._nav_follow_waypoints_name,
            callback_group=self._cb_group,
        )
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            self._nav_to_pose_name,
            callback_group=self._cb_group,
        )
        self._pick_client = ActionClient(
            self,
            Pick,
            self._pick_action_name,
            callback_group=self._cb_group,
        )
        self._locate_client = ActionClient(
            self,
            Locate,
            self._locate_action_name,
            callback_group=self._cb_group,
        )
        self._drop_client = ActionClient(
            self,
            Drop,
            self._drop_action_name,
            callback_group=self._cb_group,
        )

        self._follow_goal_future = None
        self._follow_goal_handle = None
        self._follow_result_future = None
        self._follow_cancel_future = None
        self._follow_cancel_wait_until = None

        self._nav_goal_future = None
        self._nav_goal_handle = None
        self._nav_result_future = None

        self._pick_goal_future = None
        self._pick_goal_handle = None
        self._pick_result_future = None

        self._locate_goal_future = None
        self._locate_goal_handle = None
        self._locate_result_future = None

        self._drop_goal_future = None
        self._drop_goal_handle = None
        self._drop_result_future = None

        self._detection_sub = self.create_subscription(
            String,
            self._detection_topic,
            self._on_detection,
            10,
            callback_group=self._cb_group,
        )
        self._stop_srv = self.create_service(
            Trigger,
            "/mission/stop",
            self._on_stop,
            callback_group=self._cb_group,
        )

        self._timer = self.create_timer(
            0.1, self._tick, callback_group=self._timer_cb_group
        )
        rclpy.get_default_context().on_shutdown(self._on_shutdown)

    def _set_state(self, new_state: MissionPhase) -> None:
        if self._state == new_state:
            return
        current_label = (
            "Mission Complete...IDLE"
            if self._state == MissionPhase.IDLE
            else self._state.value
        )
        next_label = (
            "Mission Complete...IDLE"
            if new_state == MissionPhase.IDLE
            else new_state.value
        )
        self.get_logger().info(f"Mission phase: {current_label} -> {next_label}")
        self._state = new_state

    def _parse_xy(self, value: Iterable, fallback: Tuple[float, float]) -> Tuple[float, float]:
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            try:
                return float(value[0]), float(value[1])
            except (TypeError, ValueError):
                return fallback
        return fallback

    def _on_detection(self, msg: String) -> None:
        if self._state != MissionPhase.PATROL_SEARCH:
            return
        detected = self._extract_detected_colour(msg)
        if detected is None:
            return
        if detected in self.detected_colours:
            return
        self.detected_colour = detected
        self.detected_colours.add(detected)
        self.get_logger().info(
            f"Qualifying detection: {self.detected_colour}; stopping patrol."
        )
        self._set_state(MissionPhase.STOP_SEARCH)

    def _extract_detected_colour(self, msg: String) -> Optional[str]:
        data = (msg.data or "").strip()
        if not data:
            return None
        try:
            payload = json.loads(data)
        except json.JSONDecodeError:
            return None

        def normalize(value: object) -> Optional[str]:
            if value is None:
                return None
            text = str(value).strip()
            if not text:
                return None
            return text.upper()

        if not isinstance(payload, dict):
            return None

        total = payload.get("total_detections")
        if isinstance(total, (int, float)) and total <= 0:
            return None

        detections = payload.get("detections") or []
        if isinstance(detections, list):
            for det in detections:
                if not isinstance(det, dict):
                    continue
                color = normalize(det.get("color_name"))
                if color:
                    return color

        summary = payload.get("summary")
        if isinstance(summary, dict):
            for key in summary.keys():
                color = normalize(key)
                if color:
                    return color

        return None

    def _tick(self) -> None:
        if self._state == MissionPhase.IDLE:
            return
        if self._state == MissionPhase.INIT:
            self.picked_total = 0
            self.detected_colour = None
            self.detected_colours.clear()
            self._set_state(MissionPhase.PATROL_SEARCH)
            return
        if self._state == MissionPhase.PATROL_SEARCH:
            self._tick_patrol()
            return
        if self._state == MissionPhase.STOP_SEARCH:
            self._tick_stop_search()
            return
        if self._state == MissionPhase.PICK:
            self._tick_pick()
            return
        if self._state == MissionPhase.LOCATE:
            self._tick_locate()
            return
        if self._state == MissionPhase.RETRIEVE_HOME:
            self._tick_retrieve_home()
            return
        if self._state == MissionPhase.DROP:
            self._tick_drop()

    def _tick_patrol(self) -> None:
        self._update_follow_goal()
        if self._follow_goal_handle is None and self._follow_goal_future is None:
            self._send_follow_goal()
        if self._follow_result_future is not None and self._follow_result_future.done():
            result = self._follow_result_future.result()
            status = result.status
            self._clear_follow_goal()
            self.get_logger().info(
                f"Patrol finished with status {status}; restarting patrol."
            )
            self._send_follow_goal()

    def _tick_stop_search(self) -> None:
        self._update_follow_goal()
        if self._follow_goal_handle is None and self._follow_goal_future is None:
            self._set_state(MissionPhase.LOCATE)
            return
        if self._follow_result_future is not None and self._follow_result_future.done():
            self._clear_follow_goal()
            self._set_state(MissionPhase.LOCATE)
            return
        if self._follow_goal_handle is not None and self._follow_cancel_future is None:
            self.get_logger().info("Canceling patrol goal.")
            self._follow_cancel_future = self._follow_goal_handle.cancel_goal_async()
        if self._follow_cancel_future is not None and self._follow_cancel_future.done():
            if self._follow_cancel_wait_until is None:
                self._follow_cancel_wait_until = time.monotonic() + 0.2
            elif time.monotonic() >= self._follow_cancel_wait_until:
                self._clear_follow_goal()
                self._set_state(MissionPhase.LOCATE)

    def _tick_locate(self) -> None:
        if not self._enable_locate:
            self.get_logger().warning("Locate disabled; skipping to PICK.")
            self._set_state(MissionPhase.PICK)
            return
        self._update_locate_goal()
        if self._locate_goal_handle is None and self._locate_goal_future is None:
            self._send_locate_goal()
        if self._locate_result_future is not None and self._locate_result_future.done():
            result = self._locate_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
            self._clear_locate_goal()
            if success:
                self.get_logger().info("Locate succeeded; proceeding to PICK.")
                self._set_state(MissionPhase.PICK)
            else:
                if self.detected_colour is not None:
                    self.detected_colours.discard(self.detected_colour)
                    self.detected_colour = None
                self.get_logger().warning("Locate failed or canceled; resuming patrol.")
                self._set_state(MissionPhase.PATROL_SEARCH)

    def _tick_pick(self) -> None:
        if not self._enable_pick:
            self.picked_total += 1
            self._rotate_global_targets()
            self.get_logger().warning(
                "Pick disabled; treating as success and skipping to RETRIEVE_HOME. "
                f"picked_total={self.picked_total}"
            )
            self._set_state(MissionPhase.RETRIEVE_HOME)
            return
        self._update_pick_goal()
        if self._pick_goal_handle is None and self._pick_goal_future is None:
            self._send_pick_goal()
        if self._pick_result_future is not None and self._pick_result_future.done():
            result = self._pick_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
            self._clear_pick_goal()
            if success:
                self.picked_total += 1
                self._rotate_global_targets()
                self.get_logger().info(
                    f"Pick succeeded. picked_total={self.picked_total}"
                )
                self._set_state(MissionPhase.RETRIEVE_HOME)
            else:
                if self.detected_colour is not None:
                    self.detected_colours.discard(self.detected_colour)
                    self.detected_colour = None
                self.get_logger().warning("Pick failed or canceled; resuming patrol.")
                self._set_state(MissionPhase.PATROL_SEARCH)

    def _tick_retrieve_home(self) -> None:
        if not self._enable_go_home:
            self.get_logger().warning("Retrieve home disabled; skipping to DROP.")
            self._set_state(MissionPhase.DROP)
            return
        self._update_nav_goal()
        if self._nav_goal_handle is None and self._nav_goal_future is None:
            self._send_home_goal()
        if self._nav_result_future is not None and self._nav_result_future.done():
            result = self._nav_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            self._clear_nav_goal()
            if success:
                self._set_state(MissionPhase.DROP)
            else:
                self.get_logger().warning("Navigate home failed; resuming patrol.")
                self._set_state(MissionPhase.PATROL_SEARCH)

    def _tick_drop(self) -> None:
        if not self._enable_drop:
            self.get_logger().warning(
                "Drop disabled; treating as success and resuming patrol."
            )
            if self.picked_total < self._target_pick_count:
                self._set_state(MissionPhase.PATROL_SEARCH)
            else:
                self._complete_mission()
            return
        self._update_drop_goal()
        if self._drop_goal_handle is None and self._drop_goal_future is None:
            self._send_drop_goal()
        if self._drop_result_future is not None and self._drop_result_future.done():
            result = self._drop_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
            self._clear_drop_goal()
            if success:
                self.get_logger().info(
                    f"Drop succeeded. picked_total={self.picked_total}"
                )
                if self.picked_total < self._target_pick_count:
                    self._set_state(MissionPhase.PATROL_SEARCH)
                else:
                    self._complete_mission()
            else:
                if self.picked_total >= self._target_pick_count:
                    self.get_logger().warning(
                        "Drop failed or canceled; target picks reached, stopping mission."
                    )
                    self._complete_mission()
                else:
                    self.get_logger().warning("Drop failed or canceled; resuming patrol.")
                    self._set_state(MissionPhase.PATROL_SEARCH)

    def _complete_mission(self) -> None:
        self.detected_colour = None
        self.detected_colours.clear()
        self._set_state(MissionPhase.IDLE)

    def _rotate_global_targets(self) -> None:
        if len(self._global_targets) <= 1:
            return
        self._global_targets = self._global_targets[1:] + self._global_targets[:1]

    def _send_follow_goal(self) -> None:
        if not self._follow_client.wait_for_server(timeout_sec=0.0):
            return
        stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"PATROL_SEARCH targets={self._global_targets}")
        poses = [
            make_pose_stamped(x, y, 0.0, self._map_frame, stamp)
            for x, y in self._global_targets
        ]
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        self._follow_goal_future = self._follow_client.send_goal_async(goal)

    def _send_home_goal(self) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=0.0):
            return
        stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(
            self._home_xy[0], self._home_xy[1], 0.0, self._map_frame, stamp
        )
        self._nav_goal_future = self._nav_client.send_goal_async(goal)

    def _send_pick_goal(self) -> None:
        if not self._pick_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warning(
                f"Pick action server not available at {self._pick_action_name}"
            )
            return
        goal = Pick.Goal()
        self._pick_goal_future = self._pick_client.send_goal_async(
            goal, feedback_callback=self._on_pick_feedback
        )

    def _send_locate_goal(self) -> None:
        if not self._locate_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warning(
                f"Locate action server not available at {self._locate_action_name}"
            )
            return
        goal = Locate.Goal()
        self._locate_goal_future = self._locate_client.send_goal_async(
            goal, feedback_callback=self._on_locate_feedback
        )

    def _send_drop_goal(self) -> None:
        if not self._drop_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warning(
                f"Drop action server not available at {self._drop_action_name}"
            )
            return
        goal = Drop.Goal()
        self._drop_goal_future = self._drop_client.send_goal_async(
            goal, feedback_callback=self._on_drop_feedback
        )

    def _update_follow_goal(self) -> None:
        if self._follow_goal_future is not None and self._follow_goal_future.done():
            goal_handle = self._follow_goal_future.result()
            self._follow_goal_future = None
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warning("Patrol goal rejected.")
                return
            self._follow_goal_handle = goal_handle
            self._follow_result_future = goal_handle.get_result_async()

    def _update_nav_goal(self) -> None:
        if self._nav_goal_future is not None and self._nav_goal_future.done():
            goal_handle = self._nav_goal_future.result()
            self._nav_goal_future = None
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warning("Navigate-to-pose goal rejected.")
                return
            self._nav_goal_handle = goal_handle
            self._nav_result_future = goal_handle.get_result_async()

    def _update_pick_goal(self) -> None:
        if self._pick_goal_future is not None and self._pick_goal_future.done():
            goal_handle = self._pick_goal_future.result()
            self._pick_goal_future = None
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warning("Pick goal rejected.")
                return
            self._pick_goal_handle = goal_handle
            self._pick_result_future = goal_handle.get_result_async()

    def _on_pick_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        stage = (feedback.stage or "").strip()
        progress = feedback.progress
        if stage:
            self.get_logger().info(f"Pick feedback: {stage} ({progress:.2f})")

    def _update_locate_goal(self) -> None:
        if self._locate_goal_future is not None and self._locate_goal_future.done():
            goal_handle = self._locate_goal_future.result()
            self._locate_goal_future = None
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warning("Locate goal rejected.")
                return
            self._locate_goal_handle = goal_handle
            self._locate_result_future = goal_handle.get_result_async()

    def _on_locate_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        stage = (feedback.stage or "").strip()
        progress = feedback.progress
        if stage:
            self.get_logger().info(f"Locate feedback: {stage} ({progress:.2f})")

    def _update_drop_goal(self) -> None:
        if self._drop_goal_future is not None and self._drop_goal_future.done():
            goal_handle = self._drop_goal_future.result()
            self._drop_goal_future = None
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warning("Drop goal rejected.")
                return
            self._drop_goal_handle = goal_handle
            self._drop_result_future = goal_handle.get_result_async()

    def _on_drop_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        stage = (feedback.stage or "").strip()
        progress = feedback.progress
        if stage:
            self.get_logger().info(f"Drop feedback: {stage} ({progress:.2f})")

    def _clear_follow_goal(self) -> None:
        self._follow_goal_future = None
        self._follow_goal_handle = None
        self._follow_result_future = None
        self._follow_cancel_future = None
        self._follow_cancel_wait_until = None

    def _clear_nav_goal(self) -> None:
        self._nav_goal_future = None
        self._nav_goal_handle = None
        self._nav_result_future = None

    def _clear_pick_goal(self) -> None:
        self._pick_goal_future = None
        self._pick_goal_handle = None
        self._pick_result_future = None

    def _clear_locate_goal(self) -> None:
        self._locate_goal_future = None
        self._locate_goal_handle = None
        self._locate_result_future = None

    def _clear_drop_goal(self) -> None:
        self._drop_goal_future = None
        self._drop_goal_handle = None
        self._drop_result_future = None

    def _on_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._cancel_all_goals()
        self.detected_colour = None
        self.detected_colours.clear()
        self._set_state(MissionPhase.IDLE)
        response.success = True
        response.message = "Mission stopped."
        return response

    def _cancel_all_goals(self) -> None:
        self._cancel_goal_handle(self._follow_goal_handle)
        self._cancel_goal_handle(self._nav_goal_handle)
        self._cancel_goal_handle(self._pick_goal_handle)
        self._cancel_goal_handle(self._locate_goal_handle)
        self._cancel_goal_handle(self._drop_goal_handle)

        self._cancel_goal_future(self._follow_goal_future)
        self._cancel_goal_future(self._nav_goal_future)
        self._cancel_goal_future(self._pick_goal_future)
        self._cancel_goal_future(self._locate_goal_future)
        self._cancel_goal_future(self._drop_goal_future)

    def _cancel_goal_handle(self, handle) -> None:
        if handle is not None:
            handle.cancel_goal_async()

    def _cancel_goal_future(self, future) -> None:
        if future is None:
            return
        if future.done():
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return

        def _cancel_when_ready(done_future):
            handle = done_future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()

        future.add_done_callback(_cancel_when_ready)

    def _on_shutdown(self) -> None:
        self._cancel_all_goals()


def main() -> None:
    rclpy.init()
    node = MissionRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
