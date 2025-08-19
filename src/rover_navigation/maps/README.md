# Maps Directory

This directory contains saved maps for the Vstone 4WDS Rover X40A navigation system.

## Map Files

Maps are saved in pairs:
- `.yaml` file: Contains map metadata (resolution, origin, etc.)
- `.pgm` file: Contains the actual occupancy grid data

## Creating Maps

To create a new map:

1. Launch SLAM mode:
```bash
ros2 launch rover_navigation rover_navigation.launch.py use_slam:=true
```

2. Drive the robot around to map the environment (either manually or with exploration)

3. Save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map_name
```

## Example Maps

- `warehouse_map.yaml/pgm`: Sample warehouse environment for testing
- `office_map.yaml/pgm`: Office environment with desks and rooms

## Map Format

Maps use the ROS standard occupancy grid format:
- Black pixels (0): Obstacles
- White pixels (255): Free space  
- Gray pixels (128): Unknown space