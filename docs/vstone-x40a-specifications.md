# Vstone 4WDS Rover X40A — Technical Specifications

This document lists the verified technical specifications for Vstone's **4WDS Rover X40A** mobile base.

## Physical Dimensions

| Parameter | Value | Notes |
|-----------|-------|------|
| **Base Shape** | Square footprint | 4WDS (Four-Wheel Drive & Steering) |
| **Width** | **384 mm** | Official overall size |
| **Depth** | **384 mm** | Official overall size |
| **Height** | **194 mm** | Spec height (ride height varies slightly with suspension) |
| **Weight** | **≈ 28.4 kg** | Base unit (increases with options) |
| **Wheel Diameter** | **140 mm** | Standard wheels (not mecanum/omni) |
| **Wheel Circumference** | **≈ 440 mm** | π × 140 mm = 439.8 mm (for odometry) |

## Drive & Steering System

- **Mobility**: **4WDS** — each of the four wheels is both **driven** and **independently steered** (omnidirectional without wheel slip).  
- **Motors**: **BLDC 40 W × 8** (drive + steering per wheel).  
- **Max speed (measured)**: **1.6 m/s**.  
- **Payload**: **≈ 40 kg**.  
- **Notes**: Uses **normal tires** for quieter, cleaner operation vs. mecanum/omni solutions.

## Control Electronics & Interfaces

- **Main control board**: **VS-WRC058** (ESP32-WROOM-32).  
- **Programming**: Arduino IDE supported (1.8.13+).  
- **Interfaces**: USB serial, **Wi-Fi (802.11 b/g/n)**, **Bluetooth (Classic & BLE 4.2)**.  
- **E-stop**: Hardware emergency stop included (mount position configurable).

## Power

| Component | Specification |
|-----------|---------------|
| **Battery** | **24 V sealed lead-acid, 288 Wh** |
| **Chassis** | Aluminum frame |

## ROS / Software

- **ROS 1**: Supported via rosserial-style messaging on the VS-WRC058 firmware.  
- **ROS 2**: **Official firmware & sample packages are provided** (micro-ROS on ESP32 + agent on PC).  
- **Recommended ROS 2 environment**: Ubuntu **22.04** + **ROS 2 Humble** (per vendor guidance).

### ROS 2 Usage (from Vstone sample)

- Launch micro-ROS Agent (serial example):  
  ```bash
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 921600 -v4
  ```

- Teleop example (remaps Nav2's /cmd_vel into the rover's input):
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/rover_twist
  ```

- Odometry is published by the sample bringup; use /odom in your stack.

Primary ROS topic used by samples: /rover_twist (geometry_msgs/Twist).

## Summary

- **Platform**: Quiet, indoor, holonomic mobile base with independent steer/drive wheels.
- **Verified specs**: 384 × 384 × 194 mm, ≈28.4 kg, Ø140 mm wheels, 1.6 m/s, ≈40 kg payload, 24 V 288 Wh SLA battery.
- **Software**: Arduino-programmable controller, ROS1/ROS2 support (micro-ROS).