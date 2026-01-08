#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveIt2 client using MoveGroup for planning + direct controller execution
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState as RobotStateMsg,
    PlanningOptions,
    MoveItErrorCodes
)
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive


class MoveIt2Client:
    """MoveIt2 client: MoveGroup for planning + direct controller for execution"""
    
    def __init__(self, node: Node, group_name: str = "arm", 
                 planning_frame: str = "base_footprint",
                 ee_link: str = "end_effector_link"):
        """
        Initialize MoveIt2 client
        
        Args:
            node: ROS2 node instance
            group_name: Planning group name
            planning_frame: Planning reference frame
            ee_link: End effector link name
        """
        self.node = node
        self.group_name = group_name
        self.planning_frame = planning_frame
        self.ee_link = ee_link
        
        # Arm joint names
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        if not self.node.has_parameter("move_group_action_name"):
            self.node.declare_parameter("move_group_action_name", "/move_action")
        self.move_group_action_name = (
            self.node.get_parameter("move_group_action_name").get_parameter_value().string_value
            or "/move_action"
        )
        if not self.node.has_parameter("move_group_send_goal_timeout"):
            self.node.declare_parameter("move_group_send_goal_timeout", 5.0)
        self.move_group_send_goal_timeout = float(
            self.node.get_parameter("move_group_send_goal_timeout")
            .get_parameter_value()
            .double_value
            or 5.0
        )
        
        # Create MoveGroup action client (for planning only)
        self.node.get_logger().info("Initializing MoveGroup action client for planning...")
        self.move_group_client = ActionClient(
            node,
            MoveGroup,
            self.move_group_action_name
        )
        
        # Create arm controller action client (for execution)
        self.node.get_logger().info("Initializing arm controller client for execution...")
        self.arm_controller_client = ActionClient(
            node,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Create IK service client
        self.node.get_logger().info("Initializing IK service client...")
        self.ik_client = self.node.create_client(GetPositionIK, '/compute_ik')
        
        self.node.get_logger().info(f"MoveIt2 client initialized for group '{group_name}'")
        self.node.get_logger().info(f"Planning frame: {planning_frame}, End effector: {ee_link}")
    
    def wait_for_action_server(self, timeout: float = 10.0) -> bool:
        """Wait for MoveGroup and arm controller action servers"""
        self.node.get_logger().info("Waiting for MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=timeout):
            error_msg = "MoveGroup action server not available after {}s timeout".format(timeout)
            self.node.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        
        self.node.get_logger().info("Waiting for arm controller action server...")
        if not self.arm_controller_client.wait_for_server(timeout_sec=timeout):
            error_msg = "Arm controller action server not available after {}s timeout".format(timeout)
            self.node.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        
        self.node.get_logger().info("All action servers ready")
        return True
    
    def plan_and_execute_joints(self, target_joint_positions: list,
                                 velocity_scaling: float = 0.25,
                                 acceleration_scaling: float = 0.20,
                                 planning_time: float = 3.0) -> tuple:
        """
        Plan and execute motion to joint positions (direct joint space planning)
        
        Args:
            target_joint_positions: List of 5 joint positions [joint1, joint2, ..., joint5]
            velocity_scaling: Velocity scaling factor
            acceleration_scaling: Acceleration scaling factor
            planning_time: Planning timeout
            
        Returns:
            Tuple of (success, error_message)
        """
        try:
            if len(target_joint_positions) != 5:
                return (False, f"Expected 5 joint positions, got {len(target_joint_positions)}")
            
            self.node.get_logger().info(
                f"Planning to joint positions: {[f'{j:.3f}' for j in target_joint_positions]}"
            )
            
            success, trajectory, error_msg = self._plan_to_joints(
                target_joint_positions, velocity_scaling, acceleration_scaling, planning_time
            )
            
            if not success:
                return (False, f"Planning failed: {error_msg}")
            
            # Execute trajectory via arm_controller
            self.node.get_logger().info("Executing planned trajectory via arm_controller...")
            
            success = self._execute_via_controller(trajectory)
            
            if success:
                self.node.get_logger().info("Motion completed successfully")
                return (True, "")
            else:
                return (False, "Execution failed")
                
        except Exception as e:
            error_msg = f"Exception: {str(e)}"
            self.node.get_logger().error(error_msg)
            return (False, error_msg)

    def plan_and_execute(self, target_pose: PoseStamped,
                         velocity_scaling: float = 0.25,
                         acceleration_scaling: float = 0.20,
                         planning_time: float = 3.0) -> tuple:
        """
        Plan motion using MoveGroup, then execute via arm_controller
        
        Args:
            target_pose: Target pose for end effector
            velocity_scaling: Velocity scaling factor
            acceleration_scaling: Acceleration scaling factor
            planning_time: Planning timeout
            
        Returns:
            Tuple of (success, error_message)
        """
        try:
            # Step 1: Plan using MoveGroup
            self.node.get_logger().info(
                f"Planning to pose: [{target_pose.pose.position.x:.3f}, "
                f"{target_pose.pose.position.y:.3f}, {target_pose.pose.position.z:.3f}]"
            )
            
            success, trajectory, error_msg = self._plan_to_pose(
                target_pose, velocity_scaling, acceleration_scaling, planning_time
            )
            
            if not success:
                return (False, f"Planning failed: {error_msg}")
            
            # Step 2: Execute trajectory via arm_controller
            self.node.get_logger().info("Executing planned trajectory via arm_controller...")
            
            success = self._execute_via_controller(trajectory)
            
            if success:
                self.node.get_logger().info("Motion completed successfully")
                return (True, "")
            else:
                return (False, "Execution failed")
                
        except Exception as e:
            error_msg = f"Exception: {str(e)}"
            self.node.get_logger().error(error_msg)
            return (False, error_msg)

    def plan_and_execute_pose_constraints(self, target_pose: PoseStamped,
                                          velocity_scaling: float = 0.25,
                                          acceleration_scaling: float = 0.20,
                                          planning_time: float = 3.0,
                                          position_tolerance: float = 0.01,
                                          orientation_tolerance: float = 0.1) -> tuple:
        """
        Plan motion using MoveGroup pose constraints, then execute via arm_controller

        Args:
            target_pose: Target pose for end effector
            velocity_scaling: Velocity scaling factor
            acceleration_scaling: Acceleration scaling factor
            planning_time: Planning timeout
            position_tolerance: Position tolerance (meters)
            orientation_tolerance: Orientation tolerance (radians)

        Returns:
            Tuple of (success, error_message)
        """
        try:
            self.node.get_logger().info(
                f"Planning to pose constraints: [{target_pose.pose.position.x:.3f}, "
                f"{target_pose.pose.position.y:.3f}, {target_pose.pose.position.z:.3f}]"
            )

            success, trajectory, error_msg = self._plan_to_pose_constraints(
                target_pose,
                velocity_scaling,
                acceleration_scaling,
                planning_time,
                position_tolerance,
                orientation_tolerance,
            )

            if not success:
                return (False, f"Planning failed: {error_msg}")

            self.node.get_logger().info("Executing planned trajectory via arm_controller...")
            success = self._execute_via_controller(trajectory)

            if success:
                self.node.get_logger().info("Motion completed successfully")
                return (True, "")
            else:
                return (False, "Execution failed")

        except Exception as e:
            error_msg = f"Exception: {str(e)}"
            self.node.get_logger().error(error_msg)
            return (False, error_msg)

    def _plan_to_joints(self, target_joint_positions: list,
                        velocity_scaling: float,
                        acceleration_scaling: float,
                        planning_time: float) -> tuple:
        """
        Plan trajectory using joint space constraints (internal method)
        
        Args:
            target_joint_positions: List of 5 joint positions
            
        Returns:
            Tuple of (success, trajectory, error_message)
        """
        try:
            # Create MoveGroup goal for planning only
            goal = MoveGroup.Goal()
            
            # Create motion plan request
            req = MotionPlanRequest()
            req.group_name = self.group_name
            req.num_planning_attempts = 5
            req.allowed_planning_time = planning_time
            req.max_velocity_scaling_factor = velocity_scaling
            req.max_acceleration_scaling_factor = acceleration_scaling
            
            # Set workspace bounds
            req.workspace_parameters.header.frame_id = self.planning_frame
            req.workspace_parameters.min_corner.x = -1.0
            req.workspace_parameters.min_corner.y = -1.0
            req.workspace_parameters.min_corner.z = -0.5
            req.workspace_parameters.max_corner.x = 1.0
            req.workspace_parameters.max_corner.y = 1.0
            req.workspace_parameters.max_corner.z = 1.5
            
            # Use joint constraints (simplest and most reliable)
            constraints = Constraints()
            
            for i, joint_name in enumerate(self.arm_joint_names):
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = target_joint_positions[i]
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
            
            req.goal_constraints = [constraints]
            
            # Start from current state
            req.start_state.is_diff = True
            
            goal.request = req
            
            # Planning options - PLAN ONLY, do NOT execute
            goal.planning_options = PlanningOptions()
            goal.planning_options.plan_only = True
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.planning_options.planning_scene_diff.robot_state.is_diff = True
            
            # Send planning goal
            if not self.move_group_client.server_is_ready():
                return (False, None, "MoveGroup action server not available")
            
            self.node.get_logger().info("🔧 Sending joint planning request to MoveGroup...")
            self.node.get_logger().info(f"   Planning attempts: {req.num_planning_attempts}")
            self.node.get_logger().info(f"   Planning time: {planning_time}s")
            self.node.get_logger().info(f"   Velocity scaling: {velocity_scaling}")
            self.node.get_logger().info(f"   Acceleration scaling: {acceleration_scaling}")
            
            send_goal_future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(
                self.node, send_goal_future, timeout_sec=self.move_group_send_goal_timeout
            )
            
            if not send_goal_future.done():
                self.node.get_logger().error("❌ Planning goal send TIMEOUT (MoveGroup not responding)")
                return (False, None, "Planning goal send timeout - MoveGroup may not be ready")
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.node.get_logger().error("❌ Planning goal REJECTED by MoveGroup")
                return (False, None, "Planning goal rejected by MoveGroup - start state may be invalid")
            
            self.node.get_logger().info("✓ Planning goal accepted, MoveGroup is computing trajectory...")
            
            # Wait for planning result
            result_future = goal_handle.get_result_async()
            actual_timeout = planning_time + 2.0
            self.node.get_logger().info(f"⏳ Waiting for planning result (timeout: {actual_timeout}s)...")
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=actual_timeout)
            
            if not result_future.done():
                self.node.get_logger().error(f"❌ Planning TIMEOUT after {actual_timeout}s")
                return (False, None, f"Planning timeout after {actual_timeout}s - MoveGroup too slow or stuck")
            
            result = result_future.result().result
            error_code = result.error_code.val
            
            # Check result
            if error_code == MoveItErrorCodes.SUCCESS:
                traj_points = len(result.planned_trajectory.joint_trajectory.points)
                self.node.get_logger().info(f"✅ Planning SUCCEEDED! Trajectory has {traj_points} waypoints")
                return (True, result.planned_trajectory, "")
            else:
                # Decode MoveIt error code
                error_explanations = {
                    -1: "FAILURE (generic failure)",
                    -2: "PLANNING_FAILED (no valid plan found)",
                    -3: "INVALID_MOTION_PLAN (plan validation failed)",
                    -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                    -5: "CONTROL_FAILED (execution control failed)",
                    -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                    -7: "TIMED_OUT (planning exceeded time limit)",
                    -10: "PREEMPTED (planning was cancelled)",
                    -11: "START_STATE_IN_COLLISION (robot starts in collision)",
                    -12: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
                    -13: "GOAL_IN_COLLISION (target pose causes collision)",
                    -14: "GOAL_VIOLATES_PATH_CONSTRAINTS",
                    -15: "GOAL_CONSTRAINTS_VIOLATED",
                    -21: "INVALID_GROUP_NAME",
                    -22: "INVALID_GOAL_CONSTRAINTS",
                    -23: "INVALID_ROBOT_STATE",
                    -24: "INVALID_LINK_NAME",
                    -25: "INVALID_OBJECT_NAME",
                    -31: "FRAME_TRANSFORM_FAILURE (TF lookup failed)",
                    -32: "COLLISION_CHECKING_UNAVAILABLE",
                    -33: "ROBOT_STATE_STALE (robot state too old)",
                    -34: "SENSOR_INFO_STALE",
                    -35: "COMMUNICATION_FAILURE",
                    -36: "CRASH (planner crashed)",
                    -37: "ABORT (planner aborted)",
                    99999: "UNKNOWN_ERROR (undefined error code)"
                }
                
                error_explanation = error_explanations.get(error_code, f"UNKNOWN ({error_code})")
                
                error_msg = f"Planning failed with MoveIt error code {error_code}: {error_explanation}"
                self.node.get_logger().error("=" * 60)
                self.node.get_logger().error(f"❌ MOVEIT PLANNING FAILED")
                self.node.get_logger().error(f"   Error Code: {error_code}")
                self.node.get_logger().error(f"   Explanation: {error_explanation}")
                self.node.get_logger().error("=" * 60)
                
                # Provide specific guidance for common errors
                if error_code == -11:
                    self.node.get_logger().error("🔍 START_STATE_IN_COLLISION detected!")
                    self.node.get_logger().error("   Possible causes:")
                    self.node.get_logger().error("   1. Robot arm is touching the ground")
                    self.node.get_logger().error("   2. Robot arm is in self-collision")
                    self.node.get_logger().error("   3. Collision geometry too large")
                elif error_code == -2 or error_code == -7:
                    self.node.get_logger().error("🔍 PLANNING_FAILED or TIMED_OUT!")
                    self.node.get_logger().error("   Possible causes:")
                    self.node.get_logger().error("   1. Target joints out of limits")
                    self.node.get_logger().error("   2. Path requires avoiding obstacles (increase planning time)")
                    self.node.get_logger().error("   3. Start and goal too far apart")
                elif error_code == -13:
                    self.node.get_logger().error("🔍 GOAL_IN_COLLISION detected!")
                    self.node.get_logger().error("   Possible causes:")
                    self.node.get_logger().error("   1. Target position causes arm to collide with environment")
                    self.node.get_logger().error("   2. Target joints outside limits")
                
                self.node.get_logger().error("=" * 60)
                return (False, None, error_msg)
                
        except Exception as e:
            error_msg = f"Planning exception: {str(e)}"
            self.node.get_logger().error("=" * 60)
            self.node.get_logger().error(f"❌ EXCEPTION in _plan_to_joints: {error_msg}")
            self.node.get_logger().error("=" * 60)
            return (False, None, error_msg)
    
    def _plan_to_pose(self, target_pose: PoseStamped,
                      velocity_scaling: float,
                      acceleration_scaling: float,
                      planning_time: float) -> tuple:
        """
        Plan trajectory using MoveGroup action (internal method)
        
        Returns:
            Tuple of (success, trajectory, error_message)
        """
        try:
            # Create MoveGroup goal for planning only
            goal = MoveGroup.Goal()
            
            # Create motion plan request
            req = MotionPlanRequest()
            req.group_name = self.group_name
            req.num_planning_attempts = 5
            req.allowed_planning_time = planning_time
            req.max_velocity_scaling_factor = velocity_scaling
            req.max_acceleration_scaling_factor = acceleration_scaling
            
            # Set workspace bounds
            req.workspace_parameters.header.frame_id = self.planning_frame
            req.workspace_parameters.min_corner.x = -1.0
            req.workspace_parameters.min_corner.y = -1.0
            req.workspace_parameters.min_corner.z = -0.5
            req.workspace_parameters.max_corner.x = 1.0
            req.workspace_parameters.max_corner.y = 1.0
            req.workspace_parameters.max_corner.z = 1.5
            
            # Use IK to compute target joint positions, then use joint constraints
            # This is more reliable than Pose constraints
            self.node.get_logger().info("Computing IK for target pose...")
            ik_success, joint_positions, ik_error = self._compute_ik(target_pose)
            
            if not ik_success:
                return (False, None, f"IK failed: {ik_error}")
            
            self.node.get_logger().info(f"IK solution: {[f'{j:.3f}' for j in joint_positions]}")
            
            # Use joint constraints (more reliable than Pose constraints)
            constraints = Constraints()
            
            for i, joint_name in enumerate(self.arm_joint_names):
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = joint_positions[i]
                jc.tolerance_above = 0.01  # 0.01 rad tolerance
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
            
            req.goal_constraints = [constraints]
            
            # Start from current state
            req.start_state.is_diff = True
            
            goal.request = req
            
            # Planning options - PLAN ONLY, do NOT execute
            goal.planning_options = PlanningOptions()
            goal.planning_options.plan_only = True  # Only plan!
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.planning_options.planning_scene_diff.robot_state.is_diff = True
            
            # Send planning goal
            if not self.move_group_client.server_is_ready():
                return (False, None, "MoveGroup action server not available")
            
            self.node.get_logger().info("Sending planning request to MoveGroup...")
            send_goal_future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(
                self.node, send_goal_future, timeout_sec=self.move_group_send_goal_timeout
            )
            
            if not send_goal_future.done():
                return (False, None, "Planning goal send timeout")
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                return (False, None, "Planning goal rejected by MoveGroup")
            
            self.node.get_logger().info("Planning goal accepted, computing trajectory...")
            
            # Wait for planning result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=planning_time + 2.0)
            
            if not result_future.done():
                return (False, None, "Planning timeout")
            
            result = result_future.result().result
            
            # Check result
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.node.get_logger().info("Planning succeeded, trajectory ready")
                return (True, result.planned_trajectory, "")
            else:
                error_msg = f"Planning failed with MoveIt error code: {result.error_code.val}"
                self.node.get_logger().error(error_msg)
                return (False, None, error_msg)
                
        except Exception as e:
            return (False, None, f"Planning exception: {str(e)}")

    def _plan_to_pose_constraints(self, target_pose: PoseStamped,
                                  velocity_scaling: float,
                                  acceleration_scaling: float,
                                  planning_time: float,
                                  position_tolerance: float,
                                  orientation_tolerance: float) -> tuple:
        """
        Plan trajectory using pose constraints (internal method)

        Returns:
            Tuple of (success, trajectory, error_message)
        """
        try:
            goal = MoveGroup.Goal()

            req = MotionPlanRequest()
            req.group_name = self.group_name
            req.num_planning_attempts = 5
            req.allowed_planning_time = planning_time
            req.max_velocity_scaling_factor = velocity_scaling
            req.max_acceleration_scaling_factor = acceleration_scaling

            req.workspace_parameters.header.frame_id = self.planning_frame
            req.workspace_parameters.min_corner.x = -1.0
            req.workspace_parameters.min_corner.y = -1.0
            req.workspace_parameters.min_corner.z = -0.5
            req.workspace_parameters.max_corner.x = 1.0
            req.workspace_parameters.max_corner.y = 1.0
            req.workspace_parameters.max_corner.z = 1.5

            constraints = Constraints()

            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = target_pose.header.frame_id
            pos_constraint.link_name = self.ee_link
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            side = max(position_tolerance * 2.0, 0.002)
            box.dimensions = [side, side, side]
            pos_constraint.constraint_region.primitives.append(box)
            pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
            pos_constraint.weight = 1.0

            ori_constraint = OrientationConstraint()
            ori_constraint.header.frame_id = target_pose.header.frame_id
            ori_constraint.link_name = self.ee_link
            ori_constraint.orientation = target_pose.pose.orientation
            ori_constraint.absolute_x_axis_tolerance = orientation_tolerance
            ori_constraint.absolute_y_axis_tolerance = orientation_tolerance
            ori_constraint.absolute_z_axis_tolerance = orientation_tolerance
            ori_constraint.weight = 1.0

            constraints.position_constraints.append(pos_constraint)
            constraints.orientation_constraints.append(ori_constraint)
            req.goal_constraints = [constraints]

            req.start_state.is_diff = True
            goal.request = req

            goal.planning_options = PlanningOptions()
            goal.planning_options.plan_only = True
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.planning_options.planning_scene_diff.robot_state.is_diff = True

            if not self.move_group_client.server_is_ready():
                return (False, None, "MoveGroup action server not available")

            self.node.get_logger().info("Sending pose-constraints planning request to MoveGroup...")
            send_goal_future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(
                self.node, send_goal_future, timeout_sec=self.move_group_send_goal_timeout
            )

            if not send_goal_future.done():
                return (False, None, "Planning goal send timeout")

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                return (False, None, "Planning goal rejected by MoveGroup")

            self.node.get_logger().info("Planning goal accepted, computing trajectory...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=planning_time + 2.0)

            if not result_future.done():
                return (False, None, "Planning timeout")

            result = result_future.result().result
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return (True, result.planned_trajectory, "")
            else:
                return (False, None, f"Planning failed with MoveIt error code: {result.error_code.val}")

        except Exception as e:
            return (False, None, f"Planning exception: {str(e)}")
    
    def _execute_via_controller(self, robot_trajectory) -> bool:
        """
        Execute trajectory by sending directly to arm_controller
        
        Args:
            robot_trajectory: RobotTrajectory from MoveGroup planning
            
        Returns:
            True if execution successful
        """
        try:
            if robot_trajectory is None:
                self.node.get_logger().error("No trajectory to execute")
                return False
            
            # Extract joint trajectory from RobotTrajectory
            joint_traj = robot_trajectory.joint_trajectory
            
            self.node.get_logger().info(
                f"Executing trajectory with {len(joint_traj.points)} points, "
                f"duration: {joint_traj.points[-1].time_from_start.sec if joint_traj.points else 0}s"
            )
            
            # Create FollowJointTrajectory goal
            controller_goal = FollowJointTrajectory.Goal()
            controller_goal.trajectory = joint_traj
            
            # Send to arm_controller
            if not self.arm_controller_client.server_is_ready():
                self.node.get_logger().error("Arm controller not ready")
                return False
            
            self.node.get_logger().info("Sending trajectory to arm_controller...")
            send_goal_future = self.arm_controller_client.send_goal_async(controller_goal)
            rclpy.spin_until_future_complete(
                self.node, send_goal_future, timeout_sec=self.move_group_send_goal_timeout
            )
            
            if not send_goal_future.done():
                self.node.get_logger().error("Failed to send goal to arm_controller")
                return False
            
            controller_goal_handle = send_goal_future.result()
            if not controller_goal_handle.accepted:
                self.node.get_logger().error("Trajectory rejected by arm_controller")
                return False
            
            self.node.get_logger().info("Trajectory accepted by arm_controller, executing...")
            
            # Wait for execution to complete
            result_future = controller_goal_handle.get_result_async()
            
            # Calculate timeout based on trajectory duration
            if joint_traj.points:
                last_point = joint_traj.points[-1]
                traj_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9
                timeout = max(traj_duration + 5.0, 20.0)  # Add 5s buffer but never drop below 20s
            else:
                timeout = 20.0
            
            self.node.get_logger().info(f"Waiting for execution (timeout: {timeout:.1f}s)...")
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout)
            
            if not result_future.done():
                self.node.get_logger().error("Trajectory execution timeout")
                return False
            
            result = result_future.result()
            
            # Check result status
            if result.status == 4:  # SUCCEEDED
                self.node.get_logger().info("Trajectory execution succeeded")
                return True
            else:
                self.node.get_logger().error(f"Execution failed with status: {result.status}")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"Execution exception: {str(e)}")
            return False
    
    def compute_ik(self, target_pose: PoseStamped, timeout: float = 0.1) -> tuple:
        """
        Compute inverse kinematics (not implemented - handled by MoveGroup)
        
        Returns:
            Tuple of (success, joint_positions, error_message)
        """
        return (True, None, "IK handled by MoveGroup planning")
    
    def get_current_pose(self) -> PoseStamped:
        """Get current end effector pose (not implemented)"""
        return None
    
    def _compute_ik(self, target_pose: PoseStamped, timeout: float = 5.0) -> tuple:
        """
        Compute IK for target pose
        
        Args:
            target_pose: Target pose for end effector
            timeout: Service call timeout
            
        Returns:
            Tuple of (success, joint_positions, error_message)
        """
        try:
            # Wait for IK service
            if not self.ik_client.wait_for_service(timeout_sec=2.0):
                return (False, None, "IK service not available")
            
            # Create IK request
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = self.group_name
            ik_request.ik_request.pose_stamped = target_pose
            ik_request.ik_request.avoid_collisions = True
            
            # Call IK service
            future = self.ik_client.call_async(ik_request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            
            if not future.done():
                return (False, None, "IK service call timeout")
            
            response = future.result()
            
            # Check result
            if response.error_code.val == MoveItErrorCodes.SUCCESS:
                # Extract joint positions for arm joints only
                joint_positions = []
                joint_names = response.solution.joint_state.name
                joint_values = response.solution.joint_state.position
                
                for joint_name in self.arm_joint_names:
                    if joint_name in joint_names:
                        idx = joint_names.index(joint_name)
                        joint_positions.append(joint_values[idx])
                    else:
                        return (False, None, f"Joint {joint_name} not in IK solution")
                
                return (True, joint_positions, "")
            else:
                if response.error_code.val == -31:
                    self.node.get_logger().error(
                        "IK frame transform failure. Pose frame: "
                        f"'{target_pose.header.frame_id}'"
                    )
                return (False, None, f"IK failed with error code: {response.error_code.val}")
                
        except Exception as e:
            return (False, None, f"IK exception: {str(e)}")
    
    def stop(self):
        """Stop current motion"""
        self.node.get_logger().warn("Stop not implemented")
