# Livox MID-360 Connection Guide

This guide provides step-by-step instructions for connecting and configuring the Livox MID-360 LiDAR with the ROS2 navigation stack.

## 1. Hardware Setup

### Physical Connection
- Connect MID-360 to computer via Ethernet cable
- Power on the MID-360 LiDAR
- Default MID-360 IP: `192.168.1.12`

### Network Configuration
Set your computer's network interface to the same subnet:
- Computer IP should be: `192.168.1.5` (as configured)
- Netmask: `255.255.255.0`

```bash
# Configure network interface (replace ethX with your interface)
# Note: Check if address is already assigned to avoid conflicts
sudo ip addr flush dev ethX
sudo ip addr add 192.168.1.5/24 dev ethX
sudo ip link set ethX up

# Verify configuration
ping -c 3 192.168.1.12
arp -n | grep 192.168.1.12
```

## 2. Dependencies Installation

### Install Livox SDK2
The SDK is already included in your workspace as a submodule:

```bash
# Build Livox-SDK2
cd src/Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

### Add Library Path
```bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib' >> ~/.bashrc
```

## 3. Configuration Files

### MID-360 Configuration
The configuration is located at `src/rover_bringup/config/livox_config.json`:

```json
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.1.5",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.5", 
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.5",
      "point_data_port": 56301,
      "imu_data_ip": "192.168.1.5",
      "imu_data_port": 56401,
      "log_data_ip": "192.168.1.5",
      "log_data_port": 56501
    }
  }
}
```

Key settings:
- **Host IP**: `192.168.1.5` (your computer)
- **LiDAR IP**: `192.168.1.12` (MID-360 default)
- **Ports**: 56100-56500 series for MID-360

## 4. Launch Process

### Method 1: Using Project Integration (Recommended)
```bash
# Source workspace
source install/setup.bash

# Launch Livox driver (hardware mode)
ros2 launch rover_bringup livox_driver.launch.py use_sim_time:=false
```

### Method 2: Direct livox_ros_driver2
```bash
# Launch MID-360 with RViz visualization
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# Or just the driver without RViz
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### Method 3: Full Navigation Stack
```bash
# Launch complete navigation with LiDAR
ros2 launch rover_navigation rover_navigation.launch.py use_slam:=true
```

## 5. Verification Steps

### Check LiDAR Connection
```bash
# Test network connectivity
ping 192.168.1.12

# Monitor point cloud data
ros2 topic echo /points --once

# Check topic list
ros2 topic list | grep -E "(livox|points)"
```

### Expected Topics
- `/points` - Point cloud data (remapped from `/livox/lidar`)
- `/livox/lidar` - Raw LiDAR data (before remapping)
- `/livox/imu` - IMU data (if enabled)

**Note**: Verify remapping is configured in your launch file:
```xml
remappings:
  - from: /livox/lidar
    to: /points
```

### RViz Visualization
1. Open RViz: `rviz2`
2. Set Fixed Frame to `livox_frame`
3. Add PointCloud2 display
4. Set topic to `/points`

## 6. Key Configuration Parameters

### Network Settings
From configuration files:
- **LiDAR IP**: `192.168.1.12` (MID-360 default)
- **Host IP**: `192.168.1.5` (your computer)
- **Command Port**: 56100/56101
- **Point Data Port**: 56300/56301
- **IMU Port**: 56400/56401

### Driver Parameters
From launch files:
- **Frame ID**: `livox_frame`
- **Publish Frequency**: 10.0 Hz
- **Data Format**: Customized pointcloud (xfer_format: 1)
- **Output Topic**: `/points` (remapped)

## 7. Troubleshooting

### Common Issues

**No point cloud data:**
- Check network configuration and LiDAR IP
- Verify Ethernet cable connection
- Ensure LiDAR is powered on

**Library errors:**
```bash
# Add library path if missing
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
```

**Connection timeout:**
- Verify network settings match configuration
- Check firewall settings
- Test basic connectivity with ping

**Wrong frame in RViz:**
- Set RViz Fixed Frame to `livox_frame`
- Check that transform tree is properly published

### Network Diagnostics
```bash
# Verify LiDAR is reachable
ping 192.168.1.12

# Check if ports are accessible
nc -zv 192.168.1.12 56100

# Monitor network traffic
sudo tcpdump -i ethX host 192.168.1.12
```

### Debug Commands
```bash
# Check transform tree
ros2 run tf2_tools view_frames

# Monitor LiDAR topics
ros2 topic hz /points
ros2 topic info /points

# Check driver logs
ros2 launch rover_bringup livox_driver.launch.py --ros-args --log-level debug
```

## 8. Finding the Livox MID-360 IP Address

If the default IP (192.168.1.12) doesn't work, use these methods to discover the device:

### Method 1: Livox Viewer (Recommended)
1. Connect sensor directly via Ethernet
2. Set PC to common Livox subnet (192.168.1.x/24)
3. Open Livox Viewer → discovers device and shows current IP
4. Can change device IP and save configuration

### Method 2: Auto-discovery via Driver
```bash
# Use broadcast code from device label in livox_config.json
# Driver will discover device and print IP in logs
ros2 launch rover_bringup livox_driver.launch.py --ros-args --log-level debug
```

### Method 3: Network Scanning
```bash
# Quick subnet scan
nmap -sn 192.168.1.0/24

# Ping sweep and check ARP table
for i in {1..254}; do ping -c1 -W1 192.168.1.$i >/dev/null && echo 192.168.1.$i; done
arp -n

# Monitor network traffic
sudo tcpdump -i ethX net 192.168.1.0/24
```

### Method 4: Check Other Common Subnets
```bash
# Try common private ranges if 192.168.1.x fails
sudo ip addr add 192.168.0.5/24 dev ethX  # Try 192.168.0.x
sudo ip addr add 10.0.0.5/24 dev ethX     # Try 10.0.0.x
```

## 9. Integration with Navigation Stack

The Livox MID-360 is fully integrated with the navigation stack:

- **Topic remapping**: `/livox/lidar` → `/points` (verify in launch file)
- **Frame ID**: `livox_frame` (connected to robot transform tree)
- **Navigation integration**: Automatic obstacle detection and path planning
- **SLAM compatibility**: Works with SLAM Toolbox for mapping
- **Config verification**: Ensure `livox_config.json` is loaded by driver launch

### Launch with Navigation
```bash
# SLAM mapping with Livox
ros2 launch rover_navigation rover_navigation.launch.py use_slam:=true

# Navigation with existing map
ros2 launch rover_navigation rover_navigation.launch.py use_slam:=false map:=/path/to/map.yaml
```

The MID-360 LiDAR provides 360° point cloud data that enables robust navigation and obstacle avoidance for the Vstone X40A rover.