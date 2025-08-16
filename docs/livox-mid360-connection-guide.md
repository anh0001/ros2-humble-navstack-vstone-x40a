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

Here's how you can determine the IP address of a Livox Mid-360 LiDAR sensor:

### 1. Default Static IP Address
- By default, every Livox Mid-360 is set to static IP mode
- Its IP address is assigned as `192.168.1.1XX`, where "XX" corresponds to the last two digits of the device's serial number
- It uses a default subnet mask of `255.255.255.0` and a default gateway of `192.168.1.1`

### 2. Viewing the Device in Livox Viewer 2
- Connect the Mid-360 to your computer using the provided M12 aviation connector and RJ-45 Ethernet (via the splitter cable), and power it on
- Launch Livox Viewer 2 (available from Livox's official website)
- In the Device Manager panel, your Mid-360 should appear, complete with its IP address
- You can even change the device's IP through the settings in the Viewer if needed

### 3. Using Network Broadcast Queries (Advanced)
- Upon powering up, the Mid-360 periodically broadcasts its device info (including IP, serial number, status) to the network—by default, this is the broadcast address
- You could use a tool like tcpdump, Wireshark, or a custom UDP broadcast script to listen for these packets and extract the IP address—especially useful if the IP has been changed or if you're managing multiple sensors
- Livox's UDP communication protocol listens on port 56000 for device queries; the sensor will reply with its information via broadcast even if it's on a different IP subnet

### Quick Reference Table

| Method | Description |
|--------|-------------|
| Default Static IP | IP = 192.168.1.1XX (XX = last two digits of serial number) |
| Livox Viewer 2 (GUI) | View and optionally change IP via device list in the viewer |
| Network Broadcast / Packet Sniffing | Capture UDP broadcasts or queries (e.g., via port 56000) to find device info |

### What Should You Do First?

If you're just trying to quickly find the IP:
1. Connect the Mid-360 directly (M12 + Ethernet RJ-45 + power)
2. Set your computer's Ethernet IP to something compatible (e.g., 192.168.1.50, subnet mask 255.255.255.0)
3. Open Livox Viewer 2 and check the Device Manager
4. The listed device row will show the current IP—use that for any configuration or SDK connections

### Legacy Discovery Methods

If Livox Viewer 2 is not available, you can use these alternative methods:

#### Network Scanning
```bash
# Quick subnet scan
nmap -sn 192.168.1.0/24

# Ping sweep and check ARP table
for i in {1..254}; do ping -c1 -W1 192.168.1.$i >/dev/null && echo 192.168.1.$i; done
arp -n

# Monitor network traffic for UDP broadcasts on port 56000
sudo tcpdump -i ethX port 56000
```

#### Auto-discovery via Driver
```bash
# Use broadcast code from device label in livox_config.json
# Driver will discover device and print IP in logs
ros2 launch rover_bringup livox_driver.launch.py --ros-args --log-level debug
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