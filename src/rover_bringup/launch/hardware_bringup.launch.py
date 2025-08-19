#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Vstone base driver with topic remapping
    vstone_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rover_bringup'),
                'launch',
                'vstone_base.launch.py'
            ])
        ]),
        launch_arguments=[('use_sim_time', use_sim_time)]
    )

    # Livox LiDAR driver
    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rover_bringup'),
                'launch',
                'livox_driver.launch.py'
            ])
        ]),
        launch_arguments=[('use_sim_time', use_sim_time)]
    )

    # Convert point cloud to LaserScan for AMCL
    pcl_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pcl_to_scan',
        parameters=[{
            'source_frame': 'lidar_link',
            'transform_tolerance': 0.1,
            'min_height': -0.20,
            'max_height': 0.80,
            'angle_min': -1.5708,  # -π/2 radians (-90 degrees) - front half circle
            'angle_max': 1.5708,   # π/2 radians (90 degrees) - front half circle
            'angle_increment': 0.00436332313,  # π/720 = 0.25°
            'scan_time': 0.1,
            'range_min': 0.25,
            'range_max': 10.0,
            'use_inf': True,
            'concurrency_level': 1
        }],
        remappings=[('cloud_in', '/points'), ('scan', '/scan')]
    )

    # Topic remapping node for /cmd_vel -> /rover_twist
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/rover_twist'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        vstone_base_launch,
        livox_driver_launch,
        pcl_to_scan,
        cmd_vel_relay
    ])