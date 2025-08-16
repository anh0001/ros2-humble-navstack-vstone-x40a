# Vstone 4WDS Rover X40A - Technical Specifications

This document provides comprehensive technical specifications for the Vstone 4WDS Rover X40A robot platform used in this ROS 2 navigation stack.

## Physical Dimensions

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Base Shape** | Square platform | Mecanum wheel configuration |
| **Width** | ≈ 420 mm | With bumper option |
| **Depth** | ≈ 420 mm | With bumper option |
| **Height** | ≈ 190 mm | Without ROS PC option |
| **Weight** | ~15 kg | Varies with options (ROS PC, sensors) |
| **Wheel Diameter** | 140 mm | Direct-drive mecanum wheels |
| **Wheel Circumference** | ≈ 440 mm | For encoder distance calculation |

## Drive & Steering System

### Motors
- **Type**: Direct-drive BLDC motors with encoders
- **Encoders**: 4096 ppr (pulses per revolution) per wheel rotation
- **High-resolution odometry**: Enables precise position tracking

### Steering Capabilities
- **Motion Type**: Omni-directional (holonomic)
- **Degrees of Freedom**: 3 (x-axis, y-axis, z-axis rotation)
- **Wheel Configuration**: Mecanum wheels for holonomic motion
- **Automatic Calibration**: Steering calibration at startup
- **Safety Features**: Speed-dependent steering limits to prevent mechanical stress

## Control Electronics

### Main Control Board
- **Model**: VS-WRC058
- **Processor**: ESP32-WROOM-32 based
- **Compatibility**: Arduino-compatible
- **Programming**: Arduino IDE support

### Hub Board
- **Model**: VS-CN028
- **Purpose**: Motor expansion and power distribution

### Connectivity Options
- **USB**: Serial communication
- **Wi-Fi**: Configurable via Arduino sketch
- **RS485**: For peripherals, powered directly by battery

### Safety Features
- **Emergency Stop**: Integrated emergency stop + power switch
- **Motor Protection**: Motor lock protection prevents overcurrent damage

## Battery & Power System

| Component | Specification |
|-----------|--------------|
| **Battery Type** | Lithium-ion pack (Vstone standard) |
| **Supported Chargers** | PCX-3000 (blue) or S120 (black) |
| **Charging Voltage** | 12V/24V (depending on version) |
| **Monitoring** | Battery voltage available via ROS topic `/rover_sensor` |

## Software & Development Environment

### Arduino Development
- **Arduino IDE**: Version 1.8.x / 2.x supported
- **ESP32-Arduino Core**: Version 2.0.13 recommended
- **Bluetooth Library**: NimBLE-Arduino v1.4.1
- **Custom Library**: vs_wrc058_fwdsrover (Vstone Arduino library)

### ROS 2 Support
- **Architecture**: micro-ROS on ESP32 + micro-ROS Agent on PC
- **ROS 2 Version**: Humble (Ubuntu 22.04 verified)
- **ROS 2 Packages**: fwdsrover_xna_ros2 (official Vstone packages)

## ROS 2 Communication Interface

### Subscribed Topics
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/rover_twist` | `geometry_msgs/Twist` | Velocity commands: linear x/y [m/s], angular z [rad/s] |

### Published Topics
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/rover_sensor` | `std_msgs/Int16MultiArray` | Bumper sensor inputs + battery voltage |
| `/rover_odo` | `geometry_msgs/Twist` | Odometry velocities (linear x, y and angular z) |

## Performance Parameters

The following parameters are configurable via XnA memory map:

| Parameter | Memory Address | Type | Description |
|-----------|----------------|------|-------------|
| **Max Translational Speed** | 0x40 | MU16_P_SLIMX | Maximum speed in mm/s |
| **Max Rotational Speed** | 0x44 | MU16_P_RLIM | Maximum rotation in mrad/s |
| **Max Acceleration/Deceleration** | 0x46 | MU16_P_ALIM | Maximum acceleration in mm/s² |
| **Motor Current Limits** | 0x38–0x3E | - | Per-channel current limits |
| **Battery Voltage** | 0x82 | MU16_M_VI | Battery voltage monitoring |

### Measured Feedback Parameters
- **Encoder Position**: 32-bit precision
- **Motor Speed**: 32-bit measurement
- **Motor Current**: 16-bit monitoring
- **Motor Temperature**: Built-in temperature sensors
- **Battery Voltage**: Real-time monitoring

## Optional ROS PC Integration

The X40A can be equipped with an optional onboard ROS PC for advanced autonomy:

| Model | Processor | RAM | Storage | GPU |
|-------|-----------|-----|---------|-----|
| **UM350** | AMD Ryzen 5 3550H | 8GB | SSD 256GB | - |
| **UM590** | AMD Ryzen 9 5900HX | 16GB | SSD 512GB | - |
| **UM760** | AMD Ryzen 5 7640HS | 32GB | SSD 512GB | - |
| **NUC14** | Intel Core Ultra 5 125H | 32GB | SSD 256GB | Intel Arc GPU |

## Navigation Stack Integration

### Frame Hierarchy
- **Odometry Frame**: `odom`
- **Base Frame**: `base_footprint` → `base_link`
- **Sensor Frames**: Attached to `base_link`

### Topic Mapping
- **Navigation Command**: `/cmd_vel` → `/rover_twist` (remapped for Vstone compatibility)
- **Odometry Input**: `/rover_odo` → `/odom` (processed by official Vstone package)
- **Hardware Communication**: `/dev/ttyACM0` at 115200 baud via micro-ROS

### Physical Parameters (Used in Navigation Tuning)
- **Wheel Base**: 0.24m (front-rear axle distance)
- **Wheel Track**: 0.175m (left-right wheel distance)
- **Wheel Radius**: 0.075m (estimated from wheel geometry)
- **Robot Radius**: 0.25m (for collision checking)

## Development Resources

### Official Documentation
- **Hardware Manual**: `docs/4wds-rover-x40a-manual-20250312.pdf`
- **XnA Datamap**: `docs/4wds-rover-xna-datamap-20221201.pdf`
- **ROS 2 Manual**: `docs/4wds-rover-ros2-manual-20250312.pdf`

### Required Libraries
- **Arduino Libraries**: Available at [Google Drive](https://drive.google.com/file/d/1tSduQFadtyptYKn2otTsEYBXKyfuHoxZ/view?usp=sharing)
- **Critical Versions**:
  - Arduino IDE with ESP32 board support version 2.0.13
  - NimBLE-Arduino library version 1.4.1
  - Vstone vs_wrc058_fwdsrover library from Google Drive link

### Git Submodules
- **Official Vstone Packages**: `fwdsrover_xna_ros2` (included as git submodule)
- **Robot Description**: `mecanumrover_description` (humble branch)

## Summary

The Vstone 4WDS Rover X40A is a professional-grade holonomic mobile robot featuring:

- **Precise Odometry**: 4096 ppr encoders enable millimeter-level position tracking
- **Holonomic Motion**: Mecanum wheels provide omnidirectional movement
- **ESP32 Control**: Arduino-compatible with Wi-Fi capability
- **ROS 2 Native**: Full micro-ROS integration with official Vstone packages
- **Configurable Performance**: Memory-mapped parameter control
- **Safety Features**: Emergency stop and motor protection
- **Expandable Platform**: Optional onboard PC for advanced autonomy

This platform is ideally suited for indoor navigation applications requiring precise positioning, omnidirectional movement, and reliable ROS 2 integration.