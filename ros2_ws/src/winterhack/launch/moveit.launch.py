import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Command-line arguments
    tutorial_arg = DeclareLaunchArgument(
        "rviz_tutorial", default_value="False", description="Tutorial flag"
    )

    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gazebo = LaunchConfiguration('use_gazebo', default='false')
    use_real = LaunchConfiguration('use_real', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Add the DeclareLaunchArgument for use_sim_time and use_real
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    use_gazebo_arg = DeclareLaunchArgument(
        "use_gazebo", default_value="false", description="Use Gazebo for simulation"
    )

    use_real_arg = DeclareLaunchArgument(
        'use_real',
        default_value='false',
        description='Load real hardware controllers if true'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz if true'
    )

    winterhack_share = get_package_share_directory("winterhack")
    urdf_path = os.path.join(winterhack_share, "config", "rosmentor.urdf.xacro")
    srdf_path = os.path.join(winterhack_share, "config", "rosmentor.srdf")
    moveit_controllers_path = os.path.join(
        winterhack_share, "config", "moveit_controllers.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder("rosmentor", package_name="winterhack")
        .robot_description(
            file_path=urdf_path,
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path=srdf_path)
        .trajectory_execution(file_path=moveit_controllers_path)
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node_gazebo = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
            "--log-level",
            "moveit_robot_model:=fatal",
            "--log-level",
            "moveit_robot_model.robot_model:=fatal",
        ],
        condition=IfCondition(use_gazebo),
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
            "--log-level",
            "moveit_robot_model:=fatal",
            "--log-level",
            "moveit_robot_model.robot_model:=fatal",
        ],
        condition=UnlessCondition(use_gazebo),
    )

    tutorial_mode = LaunchConfiguration("rviz_tutorial")

    current_dir = os.path.dirname(os.path.realpath(__file__))
    rviz_base = os.path.abspath(os.path.join(current_dir, '..', 'rviz'))

    default_rviz_config = os.path.join(rviz_base, "nav_moveit.rviz")
    rviz_empty_config = os.path.join(rviz_base, "moveit_empty.rviz")
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="RViz config file",
    )
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_node_tutorial = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        additional_env={"RCUTILS_LOGGING_SEVERITY_THRESHOLD": "40"},
        arguments=[
            "-d",
            rviz_empty_config,
            "--ros-args",
            "--log-level",
            "warn",
            "--log-level",
            "class_loader:=fatal",
            "--log-level",
            "class_loader.impl:=fatal",
            "--log-level",
            "interactive_marker_display:=warn",
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(tutorial_mode),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        additional_env={"RCUTILS_LOGGING_SEVERITY_THRESHOLD": "40"},
        arguments=[
            "-d",
            rviz_config,
            "--ros-args",
            "--log-level",
            "warn",
            "--log-level",
            "class_loader:=fatal",
            "--log-level",
            "class_loader.impl:=fatal",
            "--log-level",
            "interactive_marker_display:=warn",
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=UnlessCondition(tutorial_mode),
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {'use_sim_time': use_sim_time}],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("winterhack"),
        "config",
        "ros2_controllers.yaml",
    )
    use_local_ros2_control = PythonExpression(
        ["'", use_gazebo, "' == 'false' and '", use_real, "' == 'false'"]
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
        condition=IfCondition(use_local_ros2_control),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=IfCondition(use_local_ros2_control),
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        condition=IfCondition(use_local_ros2_control),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        condition=IfCondition(use_local_ros2_control),
    )

    # Warehouse mongodb server
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    )

    # Servo controller launch
    # servo_controller_package_path = get_package_share_directory('servo_controller')
    # servo_controller_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(servo_controller_package_path, 'launch/servo_controller.launch.py')]),
    #     condition=IfCondition(use_real)
    # )

    # Robot controller launch
    # robot_controller_package_path = get_package_share_directory('ros_robot_controller')
    # robot_controller_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(robot_controller_package_path, 'launch/ros_robot_controller.launch.py')]),
    #     condition=IfCondition(use_real)
    # )

    rviz_group = GroupAction(
        actions=[rviz_node, rviz_node_tutorial],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            declare_use_sim_time_argument,
            tutorial_arg,
            db_arg,
            ros2_control_hardware_type,
            use_gazebo_arg,
            use_real_arg,
            use_rviz_arg,
            rviz_config_arg,
            rviz_group,
            robot_state_publisher,
            move_group_node_gazebo,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
            mongodb_server_node,
            # servo_controller_launch,
            # robot_controller_launch,
        ]
    )
