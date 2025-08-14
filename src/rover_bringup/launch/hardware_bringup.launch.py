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
        launch_arguments={'use_sim_time': use_sim_time}.items()
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
        launch_arguments={'use_sim_time': use_sim_time}.items()
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
        cmd_vel_relay
    ])