# Rover Description Package

## Overview

This package provides URDF description for the **Vstone 4WDS Rover X40A** mobile robot platform. Since Vstone does not provide an official URDF model for the X40A, this package uses the `mecanumrover_description` package as a base model and adds X40A-specific modifications.

## URDF Model Structure

### Base Model: mecanumrover_description
- **Source**: Official Vstone `mecanumrover_description` package (git submodule, humble branch)
- **Base file**: `mecanum3.xacro` from the mecanumrover_description package
- **Usage**: Included via `<xacro:include filename="$(find mecanumrover_description)/urdf/mecanum3.xacro" />`

### X40A-Specific Modifications

The rover_x40a_official.urdf.xacro file extends the base mecanumrover model with:

1. **LiDAR Integration**: Livox MID-360 sensor
   - Mounted at position: `x=0.1, y=0, z=0.2` relative to base_link
   - Provides `/points` topic (sensor_msgs/PointCloud2)
   - 360° horizontal scan, 16 vertical samples

2. **ROS 2 Control Interface**: Four-wheel mecanum drive
   - Velocity control for all four wheels (front_left, front_right, back_left, back_right)
   - Velocity limits: -10 to +10 rad/s per wheel

3. **Gazebo Integration**: Simulation plugins
   - gazebo_ros2_control for hardware interface
   - Ray sensor plugin for LiDAR simulation

## Key Differences: mecanumrover vs Real X40A

### Physical Specifications
| Parameter | mecanumrover_description | Actual Vstone X40A | Notes |
|-----------|-------------------------|-------------------|-------|
| Drive Type | Mecanum wheels (4WD) | Differential drive (4WD) | X40A uses standard wheels, not mecanum |
| Wheel Base | ~0.24m | 0.24m | Similar front-rear distance |
| Wheel Track | ~0.175m | 0.175m | Similar left-right distance |
| Wheel Radius | ~0.075m | 0.075m (estimated) | Approximate from geometry |
| Base Dimensions | Generic mecanum platform | Specific X40A chassis | Visual appearance differs |

### Hardware Differences
- **Real X40A**: Uses micro-ROS communication via `/dev/ttyACM0` at 115200 baud
- **Real X40A**: Publishes `/rover_odo` topic which is converted to standard `/odom`
- **Real X40A**: Has hardware emergency stop button
- **Real X40A**: Uses standard differential drive kinematics, not mecanum kinematics

### Navigation Impact
Despite the URDF differences, the navigation system works correctly because:
- **Same kinematic constraints**: Both models have similar wheel base and track dimensions
- **Topic compatibility**: Hardware odometry is converted to standard `/odom` format
- **Frame consistency**: Both use `odom` → `base_footprint` → `base_link` hierarchy
- **Conservative parameters**: Navigation parameters account for model approximations

## Usage

### Robot State Publisher Launch
```bash
ros2 launch rover_description robot_state_publisher.launch.py
```

### In Navigation Stack
The URDF is automatically loaded by the rover_navigation launch file:
```bash
ros2 launch rover_navigation rover_navigation.launch.py
```

## Files Structure

```
rover_description/
├── CMakeLists.txt
├── package.xml
├── README.md                          # This file
├── config/
│   └── ros2_controllers.yaml         # Controller configuration
├── launch/
│   └── robot_state_publisher.launch.py
├── meshes/                           # (Currently empty)
└── urdf/
    ├── rover_x40a.gazebo.xacro       # Gazebo-specific configuration
    ├── rover_x40a.urdf.xacro         # Main URDF file
    └── rover_x40a_official.urdf.xacro # Official base + X40A modifications
```

## Dependencies

- `mecanumrover_description`: Base URDF model (via git submodule)
- `robot_state_publisher`: Publishes robot transforms
- `xacro`: URDF macro processing
- `gazebo_ros2_control`: Simulation hardware interface
- `mecanum_drive_controller`: ROS 2 control plugin

## Notes for Developers

1. **Model Accuracy**: The mecanumrover base provides a reasonable approximation for navigation purposes, but visual and kinematic details may not match the real X40A exactly.

2. **Sensor Integration**: LiDAR positioning and orientation have been tuned for the actual X40A hardware setup.

3. **Future Improvements**: If Vstone releases an official X40A URDF, this package should be updated to use it directly instead of the mecanumrover approximation.

4. **Testing**: Both simulation (Gazebo) and hardware deployment use the same URDF, ensuring consistency between development and production environments.