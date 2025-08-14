#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Vstone 4WDS Rover base driver
    vstone_driver = Node(
        package='fwdsrover_xna_ros2',
        executable='fwdsrover_xna_node',
        name='vstone_base_driver',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rover_bringup'),
                'config',
                'vstone_params.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel', '/rover_twist'),
            ('/odom', '/odom')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        vstone_driver
    ])