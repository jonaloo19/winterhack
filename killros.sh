#!/usr/bin/env bash

echo "🔪 Killing ROS 2, Gazebo, ros2_control, bridges..."

if [ -f /opt/ros/humble/setup.bash ]; then
  # Ensure ros2 is in PATH even when running with sudo.
  source /opt/ros/humble/setup.bash
fi

try_pkill() {
  pkill -f "$1" >/dev/null 2>&1 || true
}

# Kill Gazebo / Ignition
try_pkill ign_gazebo
try_pkill "ign gazebo"
try_pkill ign
try_pkill gazebo

# Kill ros2_control + controllers
try_pkill controller_manager
try_pkill gz_ros2_control
try_pkill ign_ros2_control
try_pkill ros2_control

# Kill ROS-GZ bridges
try_pkill ros_gz_bridge
try_pkill parameter_bridge

# Kill common ROS 2 nodes
try_pkill robot_state_publisher
try_pkill static_transform_publisher
try_pkill joint_state_broadcaster
try_pkill joint_state_publisher
try_pkill spawner
try_pkill "ros2 launch"
try_pkill "ros2 run"
try_pkill rviz2
try_pkill nav2
try_pkill velocity_smoother
try_pkill waypoint_follower
try_pkill slam_toolbox
try_pkill move_group
try_pkill moveit
try_pkill grasp_action_server
try_pkill landerpi_arm
try_pkill color_detection
try_pkill custom_explorer
try_pkill explorer
try_pkill lifecycle_manager
try_pkill behavior_server
try_pkill bt_navigator
try_pkill planner_server
try_pkill controller_server
try_pkill smoother_server
try_pkill ros2-daemon
try_pkill pick
try_pkill drop
try_pkill locate
try_pkill detect
try_pkill mission_runner

# Nuclear fallback (comment out if you want gentler behaviour)
try_pkill "ign gazebo"
try_pkill ign
try_pkill gazebo

# Reset ROS daemon
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop
  sleep 1
  ros2 daemon start
else
  echo "[warn] ros2 not found in PATH; skipping daemon reset."
fi

echo "✅ ROS / Gazebo reset complete"
