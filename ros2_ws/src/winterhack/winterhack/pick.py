#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pick action server - orchestrates complete pick operation
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import PoseStamped, PointStamped
from winterhack_interfaces.action import Pick
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import time
import threading

from .gripper_client import GripperClient
from .moveit2_client import MoveIt2Client

import json
from typing import List, Optional, Deque
from collections import deque
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.wait_for_message import wait_for_message
import tf2_geometry_msgs  # noqa: F401  Needed for geometry_msgs TF conversions

class _AutoGoalHandle:
    def __init__(self, node: Node):
        self._node = node

    def publish_feedback(self, _feedback):
        pass

    def abort(self):
        self._node.get_logger().warn("Auto-start pick aborted")

    def succeed(self):
        self._node.get_logger().info("Auto-start pick completed")

class PickServer(Node):
    """Action server for object picking using MoveIt2 and gripper control"""
    
    # Error codes
    ERROR_SUCCESS = 0
    ERROR_TF_FAILED = 1
    ERROR_IK_FAILED = 2
    ERROR_PLANNING_FAILED = 3
    ERROR_EXECUTION_FAILED = 4
    ERROR_GRIPPER_FAILED = 5
    
    def __init__(self):
        super().__init__('pick_server')



        # Declare parameters with defaults
        self.declare_parameters(
            namespace='',
            parameters=[
                ('auto_start', False),
                ('planning.planning_frame', 'base_footprint'),
                ('planning.group_name', 'arm'),
                ('planning.ee_link', 'end_effector_link'),
                ('planning.velocity_scaling', 0.25),
                ('planning.acceleration_scaling', 0.20),
                ('planning.planning_timeout', 3.0),
                ('planning.home_joints', [0.0, 0.83776, -2.03156, -1.19380, 0.0]),
                ('grasp_offsets.descend', [-0.01, 0.0, -0.0125]), # it was z_min + (-0.0125) = 0.023
                ('gripper.controller_name', 'gripper_controller'),
                ('gripper.joint_name', 'gripper_base_joint'),
                ('gripper.open_position', 0.0),
                ('gripper.close_position', -1.73),
                ('gripper.command_timeout', 50.0),
                ('median_window', 3),
                ('color_priority', ['GREEN', 'RED', 'BLUE', 'YELLOW']),
                ('z_min', 0.035),
                ('dz_grasp', 0.0),
                ('dx_grasp', 0.0),
                ('dy_grasp', 0.00),
                ('detection_wait_timeout', 5.0),
                ('camera_frame', 'depth_cam_link'),
                ('base_frame', 'base_footprint'),
            ]
        )

        # Load parameters (defaults apply when no YAML is provided)        
        self._load_parameters()
        self.home_joints = [0.0, 0.83776, -2.03156, -1.19380, 0.0]

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_ready = False
        self._tf_wait_start = time.monotonic()
        self._tf_wait_timer = self.create_timer(0.5, self._check_tf_ready)

        # Internal state
        self._auto_running = False
        self._auto_start_timer = None
        self._recent_targets: Deque[PoseStamped] = deque(maxlen=int(self.get_parameter("median_window").get_parameter_value().integer_value or 3))

        # Initialize components
        self.get_logger().info("Initializing components...")
        self.gripper_client = GripperClient(
            self,
            controller_name=self.gripper_controller_name,
            joint_name=self.gripper_joint_name,
            timeout=self.gripper_timeout
        )
        self.moveit_client = MoveIt2Client(
            self,
            group_name=self.group_name,
            planning_frame=self.planning_frame,
            ee_link=self.ee_link
        )


        # Wait for action servers (retry until available)
        self._gripper_ready = False
        self._moveit_ready = False
        self._server_check_timer = self.create_timer(1.0, self._check_servers)
        self._check_servers()
        
        # Create action server
        self.action_server = ActionServer(
            self,
            Pick,
            '/pick',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("Pick action server ready")
        self._log_configuration()

        self.auto_start = bool(self.get_parameter('auto_start').value)
        if self.auto_start:
            self._auto_start_timer = self.create_timer(0.1, self._auto_start_once)
    
    def _load_parameters(self):
        """Load configuration parameters from YAML"""
        self.planning_frame = self.get_parameter('planning.planning_frame').value
        self.group_name = self.get_parameter('planning.group_name').value
        self.ee_link = self.get_parameter('planning.ee_link').value
        self.velocity_scaling = self.get_parameter('planning.velocity_scaling').value
        self.acceleration_scaling = self.get_parameter('planning.acceleration_scaling').value
        self.planning_timeout = self.get_parameter('planning.planning_timeout').value
        self.home_joints = self.get_parameter('planning.home_joints').value
        
        self.descend_offset = self.get_parameter('grasp_offsets.descend').value
        
        self.gripper_controller_name = self.get_parameter('gripper.controller_name').value
        self.gripper_joint_name = self.get_parameter('gripper.joint_name').value
        self.gripper_open_pos = self.get_parameter('gripper.open_position').value
        self.gripper_close_pos = self.get_parameter('gripper.close_position').value
        self.gripper_timeout = self.get_parameter('gripper.command_timeout').value
    
    def _log_configuration(self):
        """Log current configuration"""
        self.get_logger().info("=== Grasp Configuration ===")
        self.get_logger().info(f"Planning frame: {self.planning_frame}")
        self.get_logger().info(f"Group: {self.group_name}, EE: {self.ee_link}")
        self.get_logger().info(f"Velocity scaling: {self.velocity_scaling}")
        self.get_logger().info(f"Descend offset: {self.descend_offset}")
        self.get_logger().info(f"Gripper: open={self.gripper_open_pos}, close={self.gripper_close_pos}")
        self.get_logger().info("==========================")
    
    def goal_callback(self, goal_request):
        """Handle new goal request"""
        if not self._gripper_ready or not self._moveit_ready:
            self.get_logger().warn("Rejecting pick goal: required servers unavailable")
            return GoalResponse.REJECT
        if self._auto_running:
            self.get_logger().warn("Rejecting external goal while auto_start is running")
            return GoalResponse.REJECT
        self.get_logger().info("Received grasp goal")
        return GoalResponse.ACCEPT

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

    def _check_servers(self):
        """Retry until required action servers are available."""
        if not self._gripper_ready:
            self._gripper_ready = self.gripper_client.wait_for_server(timeout=1.0)
            if not self._gripper_ready:
                self.get_logger().warn("Gripper action server not available yet")
        if not self._moveit_ready:
            try:
                self._moveit_ready = self.moveit_client.wait_for_action_server(timeout=1.0)
            except RuntimeError:
                self._moveit_ready = False
            if not self._moveit_ready:
                self.get_logger().warn("MoveGroup or arm_controller not available yet")
        if self._gripper_ready and self._moveit_ready and self._server_check_timer is not None:
            self.get_logger().info("All action servers ready")
            self._server_check_timer.cancel()
            self.destroy_timer(self._server_check_timer)
            self._server_check_timer = None

    def _run_auto_sequence(self):
        goal_handle = _AutoGoalHandle(self)
        try:
            self.execute_callback(goal_handle)
        finally:
            self._auto_running = False
    
    def cancel_callback(self, goal_handle):
        """Handle goal cancellation"""
        self.get_logger().info("Grasp goal cancelled")
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        """Execute grasp operation"""
        self.get_logger().info("=== Starting grasp execution ===")
        
        feedback = Pick.Feedback()
        result = Pick.Result()
        
        try:
            wait_timeout = float(self.get_parameter("detection_wait_timeout").value)
            detection_msg = self._wait_for_detection_info(wait_timeout)
            if detection_msg is None:
                result.success = False
                result.message = "No detection message received"
                goal_handle.abort()
                return result

            target_pose = self._pose_from_detection(detection_msg)
            if target_pose is None:
                result.success = False
                result.message = "No valid detection target available"
                goal_handle.abort()
                return result

            # Step 1: Open gripper
            feedback.stage = "Initial Position"
            feedback.progress = 0.1
            goal_handle.publish_feedback(feedback)

            # Open gripper before moving
            self.gripper_client.open_gripper(self.gripper_open_pos)
            time.sleep(1.0)
            if not self.gripper_client.open_gripper(self.gripper_open_pos):
                result.success = False
                result.message = "Failed to open gripper"
                goal_handle.abort()
                return result

            # Use longer timeout for HOME movement (initial pose may be far from HOME)
            home_timeout = max(self.planning_timeout * 2, 6.0)  # At least 6 seconds
      
            success, error_msg = self.moveit_client.plan_and_execute_joints(
                self.home_joints,
                self.velocity_scaling * 0.5,  # Slower velocity for safety
                self.acceleration_scaling * 0.5,  # Slower acceleration
                home_timeout
            )
            # Step 2: Transform target pose to planning frame
            feedback.stage = "Transforming target pose to grasp pose"
            feedback.progress = 0.3
            goal_handle.publish_feedback(feedback)
            
            self.get_logger().info(
                f"Target pose in '{target_pose.header.frame_id}': "
                f"[{target_pose.pose.position.x:.3f}, "
                f"{target_pose.pose.position.y:.3f}, "
                f"{target_pose.pose.position.z:.3f}]"
            )
            
            # Use time=0 to let MoveIt resolve transforms at latest available time.
            target_pose.header.stamp = rclpy.time.Time().to_msg()
            if not target_pose.header.frame_id:
                target_pose.header.frame_id = self.planning_frame
            self.get_logger().info(
                f"Target pose frame for IK: {target_pose.header.frame_id}"
            )
            
            # Step 3: Move to grasp pose (descend)
            feedback.stage = "Plan and execute grasp pose"
            feedback.progress = 0.4
            goal_handle.publish_feedback(feedback)

            grasp_pose = self._compute_grasp_pose(target_pose)
            success, error_msg = self.moveit_client.plan_and_execute(
                grasp_pose,
                self.velocity_scaling * 0.5,  # Slower for precision
                self.acceleration_scaling * 0.5,
                self.planning_timeout
            )
            
            if not success:
                result.success = False
                result.message = f"Grasp approach failed: {error_msg}"
                goal_handle.abort()
                return result

            # Step 4: Close gripper
            feedback.stage = "Closing gripper"
            feedback.progress = 0.6
            goal_handle.publish_feedback(feedback)
            
            #if not self.gripper_client.close_gripper(self.gripper_close_pos, duration=2.0):
            if not self.gripper_client.close_gripper(self.gripper_close_pos):    
                result.success = False
                result.message = "Failed to close gripper"
                goal_handle.abort()
                return result

            grasp_pose.pose.position.z = grasp_pose.pose.position.z + 0.001  # Lift slightly after grasp
            success, error_msg = self.moveit_client.plan_and_execute(
                grasp_pose,
                self.velocity_scaling * 0.5,  # Slower for precision
                self.acceleration_scaling * 0.5,
                self.planning_timeout
            )

            # Wait for gripper to fully close and stabilize (critical for grasping)
            self.get_logger().info("Waiting for gripper to stabilize and grip object...")
 
            # Step 7: Lift object
            feedback.stage = "Move to home"
            feedback.progress = 0.8
            goal_handle.publish_feedback(feedback)
            
            success, error_msg = self.moveit_client.plan_and_execute_joints(
                self.home_joints,
                self.velocity_scaling * 0.5,  # Slower velocity for safety
                self.acceleration_scaling * 0.5,  # Slower acceleration
                home_timeout
            )
                        
            if not success:
                result.success = False
                result.message = f"Move to home failed: {error_msg}"
                goal_handle.abort()
                return result
            
            # Success!
            feedback.stage = "completed"
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)
            
            result.success = True
            result.message = "Successfully grasped object"
            self.get_logger().info(result.message)
            
            goal_handle.succeed()
            return result
            
        except Exception as e:
            self.get_logger().error(f"Grasp execution exception: {str(e)}")
            result.success = False
            result.message = f"Exception: {str(e)}"
            goal_handle.abort()
            return result

    def _wait_for_detection_info(self, timeout_sec: float) -> Optional[String]:
        try:
            ok, msg = wait_for_message(
                String,
                self,
                "/color_detection/detection_info",
                time_to_wait=timeout_sec,
            )
        except Exception as e:
            self.get_logger().warn(f"Failed waiting for detection_info: {e}")
            return None
        if not ok:
            return None
        return msg

    def _pose_from_detection(self, msg: String) -> Optional[PoseStamped]:
        """Parse detection_info JSON and return target pose in base frame."""
        if not self._tf_ready:
            return None

        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])
        except Exception as e:
            self.get_logger().warn(f"Failed to parse detection_info: {e}")
            return None

        if not detections:
            return None

        # Build priority list
        color_priority: List[str] = [
            c.upper() for c in self.get_parameter("color_priority").get_parameter_value().string_array_value
        ]

        # Find first matching detection with 3D info
        chosen = None
        for color in color_priority:
            for det in detections:
                if str(det.get("color_name", "")).upper() == color:
                    chosen = det
                    break
            if chosen:
                break

        if not chosen:
            return None

        z_opt = chosen.get("z_3d")
        x_opt = chosen.get("x_3d")
        y_opt = chosen.get("y_3d")

        if z_opt is None or x_opt is None or y_opt is None:
            u = chosen.get("center_x")
            v = chosen.get("center_y")
            z_opt = chosen.get("depth_value")
            cx = chosen.get("frame_center_x")
            cy = chosen.get("frame_center_y")
            fx = 554.25  # focal length x (pixels)
            fy = 554.25  # focal length y (pixels)
            if u is None or v is None or z_opt is None or cx is None or cy is None:
                self.get_logger().warn("Detection missing 3D coordinates and depth; skipping")
                return None
            x_opt = (float(u) - float(cx)) * float(z_opt) / fx
            y_opt = (float(v) - float(cy)) * float(z_opt) / fy
     
        # Convert from optical frame (x right, y down, z forward) to link frame (x forward, y left, z up)
        # Here we map: x_link = z_opt, y_link = -x_opt, z_link = -y_opt
        x_link = float(z_opt)
        y_link = float(-x_opt)
        z_link = float(-y_opt)

        camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        base_frame = self.get_parameter("base_frame").get_parameter_value().string_value

        # Build PointStamped in camera frame
        pt = PointStamped()
        pt.header.frame_id = camera_frame
        # Use latest TF by stamping at time 0 to avoid future extrapolation in sim time.
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = x_link
        pt.point.y = y_link
        pt.point.z = z_link

        try:
            if not self.tf_buffer.can_transform(
                base_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            ):
                self.get_logger().warn(
                    f"TF not ready for {camera_frame} -> {base_frame}",
                    throttle_duration_sec=2.0,
                )
                return None
            transformed = self.tf_buffer.transform(
                pt, base_frame, timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None

        pose = PoseStamped()
        pose.header = transformed.header
        # Lift target in Z to account for finger offset and avoid touching ground,
        # and apply small offsets to better align the finger tips with the object center.
        z_min = self.get_parameter("z_min").get_parameter_value().double_value
        dz_grasp = self.get_parameter("dz_grasp").get_parameter_value().double_value
        dx_grasp = self.get_parameter("dx_grasp").get_parameter_value().double_value
        dy_grasp = self.get_parameter("dy_grasp").get_parameter_value().double_value

        pose.pose.position.x = transformed.point.x + dx_grasp
        pose.pose.position.y = transformed.point.y + dy_grasp
        pose.pose.position.z = max(transformed.point.z + dz_grasp, z_min)
        pose.pose.orientation.w = 1.0  # orientation handled by grasp server

        # Temporal median smoothing
        self._recent_targets.append(pose)
        xs = sorted(p.pose.position.x for p in self._recent_targets)
        ys = sorted(p.pose.position.y for p in self._recent_targets)
        zs = sorted(p.pose.position.z for p in self._recent_targets)
        mid = len(self._recent_targets) // 2
        pose_smoothed = PoseStamped()
        pose_smoothed.header = pose.header
        pose_smoothed.pose.position.x = xs[mid]
        pose_smoothed.pose.position.y = ys[mid]
        pose_smoothed.pose.position.z = zs[mid]
        pose_smoothed.pose.orientation.w = 1.0
        return pose_smoothed

    def _compute_grasp_pose(self, pregrasp_pose: PoseStamped) -> PoseStamped:
        """Compute grasp pose (descend from pregrasp)"""
        grasp = PoseStamped()
        grasp.header = pregrasp_pose.header
        grasp.pose.position.x = pregrasp_pose.pose.position.x + self.descend_offset[0]
        grasp.pose.position.y = pregrasp_pose.pose.position.y + self.descend_offset[1]
        grasp.pose.position.z = pregrasp_pose.pose.position.z + self.descend_offset[2]
        grasp.pose.orientation = pregrasp_pose.pose.orientation
        
        self.get_logger().info(
            f"Grasp pose: [{grasp.pose.position.x:.3f}, "
            f"{grasp.pose.position.y:.3f}, {grasp.pose.position.z:.3f}]"
        )
        return grasp
    
    def _check_tf_ready(self):
        camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        if self.tf_buffer.can_transform(
            base_frame,
            camera_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.1),
        ):
            self._tf_ready = True
            self.get_logger().info(f"TF ready for {camera_frame} -> {base_frame}")
            if self._tf_wait_timer is not None:
                self._tf_wait_timer.cancel()
                self.destroy_timer(self._tf_wait_timer)
                self._tf_wait_timer = None
        else:
            if (time.monotonic() - self._tf_wait_start) >= 2.0:
                self.get_logger().warn(
                    f"Waiting for TF {camera_frame} -> {base_frame}",
                    throttle_duration_sec=5.0,
                )

def main(args=None):
    rclpy.init(args=args)
    
    server = PickServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
