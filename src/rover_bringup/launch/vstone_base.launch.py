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
    
    # Micro-ROS agent for communication with Vstone hardware
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyACM0', '--baudrate', '115200', '-v4'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Vstone odometry publisher from official package
    vstone_odom = Node(
        package='fwdsrover_xna_bringup',
        executable='pub_odom',
        name='vstone_odometry',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/odom', '/odom')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        micro_ros_agent,
        vstone_odom
    ])