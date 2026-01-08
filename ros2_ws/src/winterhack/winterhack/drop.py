#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Drop action server - opens gripper to release object
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from winterhack_interfaces.action import Drop
import threading

try:
    from .gripper_client import GripperClient
except ImportError:
    from landerpi_arm.gripper_client import GripperClient


class _AutoGoalHandle:
    def __init__(self, node: Node):
        self._node = node

    def publish_feedback(self, _feedback):
        pass

    def abort(self):
        self._node.get_logger().warn("Auto-start drop aborted")

    def succeed(self):
        self._node.get_logger().info("Auto-start drop completed")


class DropServer(Node):
    """Action server for dropping objects (open gripper only)"""

    ERROR_SUCCESS = 0
    ERROR_GRIPPER_FAILED = 5

    def __init__(self):
        super().__init__('drop_server')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('auto_start', False),
                ('gripper.controller_name', 'gripper_controller'),
                ('gripper.joint_name', 'gripper_base_joint'),
                ('gripper.open_position', 0.0),
                ('gripper.command_timeout', 2.0),
            ]
        )

        self.gripper_controller_name = self.get_parameter('gripper.controller_name').value
        self.gripper_joint_name = self.get_parameter('gripper.joint_name').value
        self.gripper_open_pos = self.get_parameter('gripper.open_position').value
        self.gripper_timeout = self.get_parameter('gripper.command_timeout').value

        self._auto_running = False
        self._auto_start_timer = None

        self.gripper_client = GripperClient(
            self,
            controller_name=self.gripper_controller_name,
            joint_name=self.gripper_joint_name,
            timeout=self.gripper_timeout,
        )

        self._gripper_ready = False
        self._server_check_timer = self.create_timer(1.0, self._check_server)
        self._check_server()

        self.action_server = ActionServer(
            self,
            Drop,
            '/drop',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("Drop action server ready")

        self.auto_start = bool(self.get_parameter('auto_start').value)
        if self.auto_start:
            self._auto_start_timer = self.create_timer(0.1, self._auto_start_once)

    def goal_callback(self, goal_request):
        if self._auto_running:
            self.get_logger().warn("Rejecting external goal while auto_start is running")
            return GoalResponse.REJECT
        if not self._gripper_ready:
            self.get_logger().warn("Rejecting drop goal: gripper server unavailable")
            return GoalResponse.REJECT
        self.get_logger().info("Received drop goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Drop goal cancelled")
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

    def _check_server(self):
        """Retry until gripper action server is available."""
        if self._gripper_ready:
            return
        self._gripper_ready = self.gripper_client.wait_for_server(timeout=1.0)
        if self._gripper_ready:
            self.get_logger().info("Gripper action server ready")
            if self._server_check_timer is not None:
                self._server_check_timer.cancel()
                self.destroy_timer(self._server_check_timer)
                self._server_check_timer = None
        else:
            self.get_logger().warn("Gripper action server not available yet")

    def _run_auto_sequence(self):
        goal_handle = _AutoGoalHandle(self)
        try:
            self.execute_callback(goal_handle)
        finally:
            self._auto_running = False

    def execute_callback(self, goal_handle):
        feedback = Drop.Feedback()
        result = Drop.Result()

        feedback.stage = "Opening gripper"
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        if not self.gripper_client.open_gripper(self.gripper_open_pos):
            result.success = False
            result.message = "Failed to open gripper"
            goal_handle.abort()
            return result

        feedback.stage = "completed"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        result.success = True
        result.message = "Drop complete"
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    server = DropServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
