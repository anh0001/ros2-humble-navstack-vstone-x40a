#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get URDF via xacro - includes official mecanumrover_description + Gazebo plugins
    urdf_file = PathJoinSubstitution([
        FindPackageShare('rover_description'),
        'urdf',
        'rover_x40a_official.urdf.xacro'
    ])

    # Process the xacro file to get robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # Micro-ROS agent for communication with Vstone hardware
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/x40a_ser', '--baudrate', '921600', '-v4'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Vstone odometry publisher from official package
    vstone_odom = Node(
        package='fwdsrover_xna_bringup',
        executable='pub_odom',
        name='vstone_odometry',
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
            ('/odom', '/odom')
        ]
    )
    
    # Joint state publisher for wheel joints
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 50.0
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        micro_ros_agent,
        vstone_odom,
        joint_state_publisher
    ])