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
    
    # Livox MID-360 driver
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_driver',
        output='screen',
        parameters=[{
            'user_config_path': PathJoinSubstitution([
                FindPackageShare('rover_bringup'),
                'config',
                'livox_config.json'
            ]),
            'use_sim_time': use_sim_time,
            'frame_id': 'lidar_link',
            'lidar_bag_file_path': '',
            'xfer_format': 0,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0
        }],
        remappings=[
            ('/livox/lidar', '/points')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        livox_driver
    ])