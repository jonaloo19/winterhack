#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gripper control client using FollowJointTrajectory action
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class GripperClient:
    """Client for controlling gripper via JointTrajectoryController"""
    
    def __init__(self, node: Node, controller_name: str = "gripper_controller", 
                 joint_name: str = "gripper_base_joint", timeout: float = 2.0):
        """
        Initialize gripper client
        
        Args:
            node: ROS2 node instance
            controller_name: Name of gripper controller
            joint_name: Name of gripper joint
            timeout: Command timeout in seconds
        """
        self.node = node
        self.controller_name = controller_name
        self.joint_name = joint_name
        self.timeout = timeout
        
        # Create action client
        action_name = f"/{controller_name}/follow_joint_trajectory"
        self.action_client = ActionClient(
            node,
            FollowJointTrajectory,
            action_name
        )
        
        self.node.get_logger().info(f"Gripper client initialized: {action_name}")
    
    def wait_for_server(self, timeout: float = 5.0) -> bool:
        """
        Wait for action server to be available
        
        Args:
            timeout: Maximum wait time in seconds
            
        Returns:
            True if server available, False otherwise
        """
        self.node.get_logger().info("Waiting for gripper action server...")
        return self.action_client.wait_for_server(timeout_sec=timeout)
    
    def _send_gripper_command(self, position: float, duration: float = 1.0) -> bool:
        """
        Send gripper position command
        
        Args:
            position: Target joint position in radians
            duration: Time to reach target (seconds)
            
        Returns:
            True if successful, False otherwise
        """
        # Check if server is available
        if not self.action_client.server_is_ready():
            self.node.get_logger().error("Gripper action server not available")
            return False
        
        # Create trajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = [self.joint_name]
        
        # Single trajectory point
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        # Send goal
        self.node.get_logger().info(f"→ Sending gripper command: position={position:.3f} rad, duration={duration:.1f}s")
        effective_timeout = max(self.timeout, duration + 2.0)
        self.node.get_logger().info(
            f"  Waiting for gripper controller response (timeout={effective_timeout}s)..."
        )
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=effective_timeout)
        
        if not send_goal_future.done():
            self.node.get_logger().error("✗ Gripper goal send TIMEOUT - controller may not be responding")
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Gripper goal rejected")
            return False
        
        self.node.get_logger().info("✓ Gripper goal accepted, executing...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        self.node.get_logger().info(
            f"  Waiting for execution to complete (timeout={effective_timeout}s)..."
        )
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=effective_timeout)
        
        if not result_future.done():
            self.node.get_logger().error(
                f"✗ Gripper execution TIMEOUT after {effective_timeout}s"
            )
            return False
        
        result = result_future.result()
        if result is None:
            self.node.get_logger().error("Gripper command returned None result")
            return False
            
        if result.status == 4:  # SUCCEEDED
            self.node.get_logger().info("Gripper command succeeded")
            return True
        else:
            self.node.get_logger().error(f"Gripper command failed with status: {result.status}")
            return False
    
    def open_gripper(self, open_position: float = 0.0, duration: float = 1.0) -> bool:
        """
        Open gripper to specified position
        
        Args:
            open_position: Open position in radians (default 0.0 for fully open)
            duration: Time to open (seconds)
            
        Returns:
            True if successful
        """
        self.node.get_logger().info(f"Opening gripper to {open_position:.3f} rad")
        return self._send_gripper_command(open_position, duration)
    
    def close_gripper(self, close_position: float = -1.638, duration: float = 1.0) -> bool:
        """
        Close gripper to specified position
        
        Args:
            close_position: Close position in radians (default -1.638 for fully closed)
            duration: Time to close (seconds)
            
        Returns:
            True if successful
        """
        self.node.get_logger().info(f"Closing gripper to {close_position:.3f} rad")
        return self._send_gripper_command(close_position, duration)
    
    def set_gripper(self, position: float, duration: float = 1.0) -> bool:
        """
        Set gripper to arbitrary position
        
        Args:
            position: Target position in radians [-1.638, 0.0]
            duration: Time to reach position (seconds)
            
        Returns:
            True if successful
        """
        # Clamp position to valid range
        position = max(-1.638, min(0.0, position))
        self.node.get_logger().info(f"Setting gripper to {position:.3f} rad")
        return self._send_gripper_command(position, duration)
