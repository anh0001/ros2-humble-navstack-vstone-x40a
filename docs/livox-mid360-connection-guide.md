# Livox MID-360 Connection Guide

## 1. Hardware Setup

### Physical Connection
- Connect MID-360 to computer via Ethernet cable
- Power on the MID-360 LiDAR
- MID-360 uses static IP: `192.168.1.125` (last two digits from device serial: 25)

### Network Configuration
```bash
# Configure network interface (replace ethX with your interface)
sudo ip addr flush dev ethX
sudo ip addr add 192.168.1.5/24 dev ethX
sudo ip link set ethX up

# Verify connection
ping -c 3 192.168.1.125
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
Update `src/rover_bringup/config/livox_config.json` with your device IP:

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
- **LiDAR IP**: `192.168.1.125` (device serial ending in 25)

## 4. Launch and Verification

```bash
# Source workspace
source install/setup.bash

# Launch Livox driver
ros2 launch rover_bringup livox_driver.launch.py use_sim_time:=false

# Verify connection
ping 192.168.1.125
ros2 topic echo /points --once

# Launch with navigation
ros2 launch rover_navigation rover_navigation.launch.py use_slam:=true
```