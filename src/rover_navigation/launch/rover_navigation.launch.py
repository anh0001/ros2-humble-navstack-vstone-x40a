#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Whether to run SLAM or localization mode'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('rover_navigation'),
            'maps',
            'x40a_lab.yaml'
        ]),
        description='Path to the map file'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rover_navigation'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Path to the nav2 parameters file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    enable_waypoints_arg = DeclareLaunchArgument(
        'enable_waypoints',
        default_value='false',
        description='Launch waypoint follower'
    )

    # Get launch configurations
    use_slam = LaunchConfiguration('use_slam')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_waypoints = LaunchConfiguration('enable_waypoints')

    # Robot description is now handled by vstone_base.launch.py

    # Hardware bringup (Vstone base + Livox LiDAR)
    hardware_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rover_bringup'),
                'launch',
                'hardware_bringup.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # SLAM Toolbox (only when use_slam=true)
    slam_launch = GroupAction(
        condition=IfCondition(use_slam),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('slam_toolbox'),
                        'launch',
                        'online_async_launch.py'
                    ])
                ]),
                launch_arguments={
                    'slam_params_file': PathJoinSubstitution([
                        FindPackageShare('rover_navigation'),
                        'config',
                        'slam_params.yaml'
                    ]),
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )

    # Map server and AMCL (only when use_slam=false)
    localization_launch = GroupAction(
        condition=UnlessCondition(use_slam),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}
                ]
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl']
                }]
            )
        ]
    )

    # Nav2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )
    
    # Delay Nav2 a bit so SLAM has time to publish /map and map→odom
    nav2_bringup_delayed = TimerAction(period=3.0, actions=[nav2_bringup_launch])

    # Optional waypoint follower
    waypoint_follower = Node(
        package='rover_waypoints',
        executable='waypoint_follower.py',
        name='waypoint_follower',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_waypoints)
    )

    return LaunchDescription([
        use_slam_arg,
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        enable_waypoints_arg,
        hardware_bringup_launch,
        slam_launch,
        localization_launch,
        nav2_bringup_delayed,
        waypoint_follower
    ])