# ROS 2 Humble Navigation Stack for Vstone 4WDS Rover X40A

![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg) ![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)

Production-ready autonomous navigation system for indoor delivery operations.

## Overview

This repository provides a complete ROS 2 Humble navigation stack for the Vstone 4WDS Rover X40A, designed for indoor warehouse delivery applications. The system supports both simulation and hardware deployment with a single-command launch interface.

## Features

- **One-command launch** with `use_slam` argument for mapping vs. navigation modes
- **Production-ready Nav2 configuration** with tuned parameters for X40A
- **Livox MID-360 LiDAR integration** with 3D point cloud processing
- **Gazebo simulation** with realistic warehouse environment
- **Hardware abstraction** supporting both simulation and real robot
- **Safety features** including collision monitoring and emergency stop
- **Waypoint following** with automated delivery route capabilities

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Desktop
- Nav2 navigation stack
- Gazebo Classic (for simulation)

## Installation

```bash
# Install ROS 2 Humble and Nav2
sudo apt update
sudo apt install ros-humble-desktop ros-humble-nav2-bringup

# Install Gazebo Classic
sudo apt update
sudo apt install \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins

# Install additional dependencies
sudo apt install ros-humble-slam-toolbox \
                 ros-humble-robot-localization \
                 ros-humble-pointcloud-to-laserscan \
                 ros-humble-controller-manager \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-mecanum-drive-controller \
                 ros-humble-gazebo-ros-pkgs \
                 ros-humble-gazebo-ros2-control \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-micro-ros-agent

# Clone and build workspace
git clone <repository-url> ros2-humble-navstack-vstone-x40a
cd ros2-humble-navstack-vstone-x40a

# Initialize submodules (required for Vstone packages)
git submodule update --init --recursive

# Build and install Livox SDK2 (required for Livox LiDAR driver)
cd src/Livox-SDK2
mkdir build && cd build
cmake ..
make -j4
sudo make install

# Return to workspace root and build
cd /path/to/ros2-humble-navstack-vstone-x40a
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Quick Start

### Simulation

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch rover_gazebo gazebo_sim.launch.py

# Terminal 2: Launch navigation (mapping mode)
ros2 launch rover_navigation rover_navigation.launch.py use_sim_time:=true use_slam:=true

# Terminal 3: Run waypoint demo
ros2 run rover_waypoints waypoint_follower.py
```

### Hardware

```bash
# Navigation with pre-built map
ros2 launch rover_navigation rover_navigation.launch.py \
  use_slam:=false \
  map:=/path/to/your/map.yaml

# Mapping mode  
ros2 launch rover_navigation rover_navigation.launch.py use_slam:=true
```

## Package Structure

```
src/
├── rover_description/        # URDF models and robot description (uses mecanumrover base + X40A modifications)
├── rover_navigation/         # Nav2 configuration and launch files  
├── rover_bringup/           # Hardware drivers and bringup
├── rover_gazebo/            # Simulation worlds and models
├── rover_waypoints/         # Waypoint following examples
├── fwdsrover_xna_ros2/      # Git submodule: Vstone hardware packages
├── mecanumrover_description/ # Git submodule: Base URDF model (no official X40A URDF available)
├── Livox-SDK2/              # Livox SDK2 for LiDAR hardware interface
└── livox_ros_driver2/       # ROS 2 driver for Livox MID-360 LiDAR
```

## Configuration

### Key Parameters

The navigation stack is tuned for the X40A's specifications:

- **Safety**: 30cm inflation radius ensures 5cm minimum clearance
- **Speed**: 0.4 m/s maximum for safe indoor operation
- **Accuracy**: 8cm goal tolerance meets delivery requirements
- **Sensor**: Livox MID-360 with ground filtering and 3D→2D projection

### Hardware Integration

- **Vstone Base**: Interfaces with `fwdsrover_xna_ros2` git submodule package
- **LiDAR**: Livox MID-360 via `livox_ros_driver2`
- **Topic Remapping**: `/cmd_vel` → `/rover_twist` for Vstone compatibility
- **Emergency Stop**: Hardware button provides immediate motor cutoff
- **Robot Models**: Uses `mecanumrover_description` URDF as base (no official X40A URDF available from Vstone)

## Usage Modes

### 1. Mapping (SLAM)

Create maps of new environments:

```bash
# Start SLAM
ros2 launch rover_navigation rover_navigation.launch.py use_slam:=true

# Drive manually to explore
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=rover_twist

# Save completed map
ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse_map
```

### 2. Navigation (AMCL)

Navigate using pre-built maps:

```bash
# Start localization
ros2 launch rover_navigation rover_navigation.launch.py \
  use_slam:=false \
  map:=~/maps/warehouse_map.yaml

# Set initial pose in RViz, then send navigation goals
```

### 3. Autonomous Delivery

Run automated waypoint missions:

```bash
# Start navigation system
ros2 launch rover_navigation rover_navigation.launch.py use_slam:=false

# Execute delivery route
ros2 run rover_waypoints waypoint_follower.py
```

## Monitoring and Visualization

Launch RViz for system monitoring:

```bash
rviz2 -d rviz/navigation.rviz
```

Key displays:
- Map and costmaps overlay
- Robot model with live TF frames
- LiDAR point cloud visualization  
- Global and local path planning
- Goal setting and pose estimation tools

## Performance Specifications

- **Navigation Accuracy**: ±8cm positional tolerance
- **Operating Speed**: 0.4 m/s maximum, 0.05 m/s minimum
- **Safety Clearance**: 5cm minimum from obstacles
- **Continuous Operation**: Validated for 2+ hour missions
- **Environment**: Indoor, flat surfaces, 10m × 10m areas

## Troubleshooting

### Robot Doesn't Move
- Check odometry: `ros2 topic echo /odom`
- Verify TF tree: `ros2 run tf2_tools view_frames`
- Ensure Vstone base connection and power

### Livox LiDAR Issues
- Check LiDAR connection: `ros2 topic echo /points`
- Verify SDK2 installation: `ls /usr/local/lib/liblivox_lidar_sdk_*`
- Check driver launch: `ros2 launch rover_bringup livox_driver.launch.py`

### Poor Localization  
- Set initial pose in RViz near actual position
- Check LiDAR data: `ros2 topic echo /points`
- Verify map quality and sensor calibration

### Navigation Failures
- Check costmaps for false obstacles in RViz
- Adjust inflation parameters if too conservative
- Ensure goals are reachable and not in obstacles

## Safety Features

- **Hardware E-Stop**: Immediate motor cutoff via physical button
- **Collision Monitor**: Real-time obstacle detection and avoidance
- **Conservative Tuning**: Safe speeds and generous clearances
- **Recovery Behaviors**: Automatic unsticking and path replanning
- **Timeout Protection**: Mission abort on prolonged failures

## Development

### Adding New Waypoints

Edit `rover_waypoints/scripts/waypoint_follower.py`:

```python
waypoints.append(self.create_pose(x, y, yaw))
```

### Tuning Parameters

Modify `rover_navigation/config/nav2_params.yaml`:
- Adjust `inflation_radius` for clearance requirements
- Tune `desired_linear_vel` for speed vs. safety balance  
- Configure `xy_goal_tolerance` for accuracy needs

### Custom Launch Configurations

Create new launch files in `rover_bringup/launch/` for specific deployment scenarios.

## License

Apache 2.0 - See LICENSE file for details.