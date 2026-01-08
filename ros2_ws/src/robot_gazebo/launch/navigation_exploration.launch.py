import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description():
    robot_gazebo_dir = get_package_share_directory('robot_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    publish_robot_description = LaunchConfiguration('publish_robot_description')
    map_yaml = LaunchConfiguration('map')
    use_slam = LaunchConfiguration('use_slam')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [robot_gazebo_dir, 'config', 'nav2_params_exploration.yaml']),
        description='Full path to the ROS2 parameters file to use for nav2 nodes')

    declare_publish_robot_description_cmd = DeclareLaunchArgument(
        'publish_robot_description',
        default_value='false',
        description='Publish robot_description for RViz')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file. If empty, SLAM is used.')

    declare_use_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='auto',
        description="Use SLAM: true/false/auto (auto when map is empty)")

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gazebo_dir, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(
            PythonExpression([
                "'", use_slam, "' == 'true' or (",
                "'", use_slam, "' == 'auto' and '", map_yaml, "' == '' )"
            ])
        ))

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gazebo_dir, 'launch', 'localization.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_yaml,
        }.items(),
        condition=IfCondition(PythonExpression(["'", map_yaml, "' != ''"])))

    nav_exploration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gazebo_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'publish_robot_description': publish_robot_description,
            'map': map_yaml,
        }.items())

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_publish_robot_description_cmd,
        declare_map_cmd,
        declare_use_slam_cmd,
        slam_launch,
        localization_launch,
        nav_exploration_launch,
    ])
