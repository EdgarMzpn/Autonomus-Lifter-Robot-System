import launch
import launch_ros.actions

def generate_launch_description():
    # Static transform publisher node
    static_transform_publisher_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_joint',
        arguments=['odomPose_x', 'odomPose_y', 'odomPose_z', 'odomPose_roll', 'odomPose_pitch', 'odomPose_yaw', 'map', 'odom']
    )

    # Argument for puzzlebot_sim_model
    puzzlebot_sim_model_arg = launch.actions.DeclareLaunchArgument(
        name='puzzlebot_sim_model',
        default_value='$(find localisation)/urdf/puzzlebot_jetson_ed_urdf.urdf',
        description='Path to the URDF file of the puzzlebot'
    )

    # Parameter for robot_description
    robot_description_param = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='simulated_state_publisher',
        output='screen',
        parameters=[{'robot_description': launch.substitutions.TextSubstitution([
            'cat', launch.substitutions.LaunchConfiguration('puzzlebot_sim_model')
        ])}]
    )

    # RViz node
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', launch.substitutions.LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': 'true'}]  # Assuming you want to use simulated time
    )

    # Create launch description and populate
    ld = launch.LaunchDescription()

    # Add actions to launch description
    ld.add_action(static_transform_publisher_node)
    ld.add_action(puzzlebot_sim_model_arg)
    ld.add_action(robot_description_param)
    ld.add_action(rviz_node)

    return ld
