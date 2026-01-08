from launch import LaunchDescription,LaunchService
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    use_sim_time_cfg = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time = use_sim_time_cfg.perform(context)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    nav = LaunchConfiguration('nav', default='false').perform(context)
    nav_arg = DeclareLaunchArgument('nav',default_value=nav)

    remappings_default = [("/odom/tf", "tf")]
    if nav == 'true':
        remappings_default += [("/controller/cmd_vel", "/cmd_vel")]

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
                # Velocity command (ROS2 -> IGN)
                '/controller/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                # Odometry (IGN -> ROS2)
                '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                # TF (IGN -> ROS2)
                '/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                # Clock (IGN -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                # Lidar (IGN -> ROS2)
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                # IMU (IGN -> ROS2)
                '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                # Camera (IGN -> ROS2)
                '/depth_cam/depth_cam@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/depth_cam/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/depth_cam/rgb/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                '/depth_cam/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                ],
        remappings=remappings_default,
        parameters=[{'use_sim_time': use_sim_time_cfg}],
        output='screen'
    )


    map_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='screen',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
                        parameters=[{'use_sim_time': use_sim_time_cfg}])
    depth_cam_static_tf = Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='depth_cam_static_tf',
                        output='screen',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'depth_cam_link', 'robot/link4/depth_cam'],
                        parameters=[{'use_sim_time': use_sim_time_cfg}])
    rgb_cam_static_tf = Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='rgb_cam_static_tf',
                        output='screen',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'depth_cam_link', 'robot/link4/rgb_cam'],
                        parameters=[{'use_sim_time': use_sim_time_cfg}])
    return [
        use_sim_time_arg,
        bridge,
        map_static_tf,
        depth_cam_static_tf,
        rgb_cam_static_tf
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])



if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
