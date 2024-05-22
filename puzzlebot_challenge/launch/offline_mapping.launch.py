import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    puzzlebot_pkg = get_package_share_directory('puzzlebot_challenge')
    
    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])
    
    teleop_type = DeclareLaunchArgument(
        'teleop_type', default_value="keyboard", description="how to teleop ('keyboard', 'joystick' or 'none')")
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(puzzlebot_pkg,
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([puzzlebot_pkg, '/launch/rplidar.launch.py']))

    offline_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch/offline_launch.py']))

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch/rviz_launch.py']),
        condition=IfCondition(LaunchConfiguration('use_rviz')))
    
    tf_solution = Node(
        package = 'puzzlebot_challenge',
        executable = 'tf_broadcaster',
        name = 'tf_broadcaster',
    )

    tf_base_laser = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'static_transform_publisher_base_to_laser',
        arguments = ['0', '0', '0', '0', '0', '0', "base_link", "laser"]
    )

    tf_base_odom = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'static_transform_publisher_base_to_odom',
        arguments = ['0', '0', '0', '0', '0', '0', "base_link", "odom"]
    )

    return LaunchDescription([
        rviz_param,
        teleop_type,
        declare_use_sim_time_argument,
        declare_slam_params_file_cmd,
        rplidar_launch,
        offline_mapping_launch,
        rviz_launch,
        tf_solution,
        tf_base_laser,
        tf_base_odom,
    ])