import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    winterhack_dir = get_package_share_directory("winterhack")

    robot_mode = LaunchConfiguration("robot_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    publish_robot_description = LaunchConfiguration("publish_robot_description")
    map_yaml = LaunchConfiguration("map")
    use_slam = LaunchConfiguration("use_slam")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_real = LaunchConfiguration("use_real")
    moveit_rviz = LaunchConfiguration("moveit_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    cmd_vel_out = LaunchConfiguration("cmd_vel_out")

    use_sim_time_value = PythonExpression(
        ["'false' if '", robot_mode, "' == 'real' else '", use_sim_time, "'"]
    )
    use_gazebo_value = PythonExpression(
        ["'false' if '", robot_mode, "' == 'real' else '", use_gazebo, "'"]
    )
    use_real_value = PythonExpression(
        ["'true' if '", robot_mode, "' == 'real' else '", use_real, "'"]
    )
    cmd_vel_out_value = PythonExpression(
        ["'cmd_vel' if '", robot_mode, "' == 'real' else '", cmd_vel_out, "'"]
    )

    declare_robot_mode = DeclareLaunchArgument(
        "robot_mode",
        default_value="sim",
        description="Robot mode: sim or real",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [winterhack_dir, "config", "nav2_params_exploration.yaml"]
        ),
        description="Full path to the ROS2 parameters file to use for nav2 nodes",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_cmd_vel_out = DeclareLaunchArgument(
        "cmd_vel_out",
        default_value="controller/cmd_vel",
        description="Final cmd_vel topic for the base (sim or real robot)",
    )
    declare_publish_robot_description = DeclareLaunchArgument(
        "publish_robot_description",
        default_value="false",
        description="Publish robot_description for RViz",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to map yaml file. If empty, SLAM is used.",
    )
    declare_use_slam = DeclareLaunchArgument(
        "use_slam",
        default_value="true",
        description="Use SLAM: true/false/auto (auto when map is empty)",
    )
    declare_use_gazebo = DeclareLaunchArgument(
        "use_gazebo",
        default_value="true",
        description="Use Gazebo for MoveIt demo",
    )
    declare_use_real = DeclareLaunchArgument(
        "use_real",
        default_value="false",
        description="Load real hardware controllers if true",
    )
    declare_moveit_rviz = DeclareLaunchArgument(
        "moveit_rviz",
        default_value="true",
        description="Launch MoveIt RViz if true",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [winterhack_dir, "rviz", "nav_moveit.rviz"]
        ),
        description="RViz config file for the combined view",
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(winterhack_dir, "launch", "slam.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time_value}.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_slam,
                    "' == 'true' or (",
                    "'",
                    use_slam,
                    "' == 'auto' and '",
                    map_yaml,
                    "' == '' )",
                ]
            )
        ),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(winterhack_dir, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time_value,
            "params_file": params_file,
            "map": map_yaml,
        }.items(),
        condition=IfCondition(PythonExpression(["'", map_yaml, "' != ''"])),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(winterhack_dir, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time_value,
            "params_file": params_file,
            "publish_robot_description": publish_robot_description,
            "map": map_yaml,
            "use_rviz": "false",
            "cmd_vel_out": cmd_vel_out_value,
        }.items(),
    )

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(winterhack_dir, "launch", "moveit.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time_value,
            "use_gazebo": use_gazebo_value,
            "use_real": use_real_value,
            "use_rviz": moveit_rviz,
            "rviz_config": rviz_config,
        }.items(),
    )

    detection_node = Node(
        package="winterhack",
        executable="detect",
        output="screen",
        parameters=[{"show_debug_window": True, "use_sim_time": use_sim_time_value}],
    )
    locate_node = Node(
        package="winterhack",
        executable="locate",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_value, "cmd_vel_topic": cmd_vel_out_value}],
    )
    pick_node = Node(
        package="winterhack",
        executable="pick",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_value}],
    )
    drop_node = Node(
        package="winterhack",
        executable="drop",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_value}],
    )

    return LaunchDescription(
        [
            declare_robot_mode,
            declare_use_sim_time,
            declare_cmd_vel_out,
            declare_params_file,
            declare_publish_robot_description,
            declare_map,
            declare_use_slam,
            declare_use_gazebo,
            declare_use_real,
            declare_moveit_rviz,
            declare_rviz_config,
            slam_launch,
            localization_launch,
            navigation_launch,
            moveit_demo,
            detection_node,
            locate_node,
            pick_node,
            drop_node,
        ]
    )
