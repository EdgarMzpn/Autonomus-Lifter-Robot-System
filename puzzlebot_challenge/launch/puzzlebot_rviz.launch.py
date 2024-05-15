from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


def generate_launch_description():
    urdf_model_path = get_package_share_directory('puzzlebot_challenge') + '/urdf/puzzlebot.urdf'
    rviz_config_path = get_package_share_directory('puzzlebot_challenge') + '/rviz/manipulator.rviz'

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='simulated_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_model_path).read()}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    tf2_ros = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_joint',
            arguments=['odomPose_x', 'odomPose_y', 'odomPose_z', 'odomPose_roll', 'odomPose_pitch', 'odomPose_yaw', 'map', 'odom']
        )
    
    pose_sim = Node(
            package='puzzlebot_challenge',
            executable='pose_sim',
            name='puzzlebot_kinematics'
    )

    localisation = Node(
            package = 'puzzlebot_challenge',
            executable = 'odometry',
            name = 'localisation'
    )

    joint_state = Node(
            package='puzzlebot_challenge',
            executable='joint_state_publisher',
            name='joint_state_publisher'
    )

    controller = Node(
            package='puzzlebot_challenge',
            executable='velocity_control',
            name='velocity_control'
    )

    trayectory = Node(
        package='puzzlebot_challenge',
        executable='trajectory_control',
        name='trajectory_control'
    )

    simlulation = Node(
        package='puzzlebot_challenge',
        executable='obstacule_sim',
        name='obstacule_sim'
    )


    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        tf2_ros,
        #pose_sim, 
        localisation,
        joint_state, 
        controller,
        #trayectory,
        simlulation
    ])

if __name__ == '__main__':
    generate_launch_description()