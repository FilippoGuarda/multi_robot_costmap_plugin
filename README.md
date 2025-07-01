# DISCLAIMER

This is still very much a work in progress, if you want to contribute please check the [Contributing Section](#contributing).  

There are some features that I intend to add once the project is stable enough, for now I will only focus on application on Humble,
some help porting this plugin to newer ROS versions would be highly appreciated.

## Features that still need to be added

- **Obstacle Permanence**: Currently the obstacles get cleaned out once none of the robots can see them anymore, ideally the obstacle detections should remain until a robots clears them out. Much like how it currently works with the nav2 obstacle layer
- **Nav2 Goal namespace selection tool for Rviz**: Right now the only way to graphically set a goal is to manually modify the 2D goal pose functionality on rviz from the Toop Properties panel, while it's not possible to do the same with the NAV2 Goal tool.
- **Secret one**: I'm working on the concept of this one and if it works it could become a paper, so at the moment it will stay a secret.

# Multi-Robot Costmap Plugin for Nav2

A comprehensive ROS2 package that provides multi-robot obstacle sharing capabilities for the Nav2 navigation stack. This plugin enables robots in a fleet to share detected obstacles with each other, improving collective navigation awareness and safety.

## Overview

The Multi-Robot Costmap Plugin consists of two main components:

1. **MultiRobotLayer**: A Nav2 costmap plugin that integrates shared obstacle data into individual robot costmaps
2. **GlobalCostmapFusion**: A standalone ROS2 node that collects laser scan data from multiple robots and publishes a unified obstacle grid

## Features

- **Real-time obstacle sharing** between multiple robots
- **TF2-based robot pose tracking** for accurate coordinate transformations
- **Automatic map parameter detection** from map server or service
- **Configurable robot exclusion zones** to prevent robots cross-detection and controller errors
- **Thread-safe operation** with mutex-protected data structures
- **Robust error handling** with transform timeout management
- **Compatible with Nav2 lifecycle management**

## Package Structure

```
multi_robot_costmap_plugin/
├── CMakeLists.txt
├── package.xml
├── plugin.xml                          # Plugin registration file
├── include/
│   └── multi_robot_costmap_plugin/
│       ├── global_costmap_fusion.hpp   # GlobalCostmapFusion node header
│       └── multi_robot_layer.hpp       # MultiRobotLayer plugin header
├── src/
│   ├── global_costmap_fusion.cpp       # GlobalCostmapFusion implementation
│   ├── global_costmap_fusion_node.cpp  # Node executable wrapper
│   └── multi_robot_layer.cpp           # MultiRobotLayer implementation
├── config/
│   └── multi_robot_costmap.yaml        # Shared costmap configuration example
└── launch/
    └── examples_launch.py              # Demo launch file
```

## Dependencies

### System Requirements
- ROS2 Humble (tested) or later (not sure)
- Nav2 navigation stack
- TF2 for coordinate transformations

### ROS2 Package Dependencies
- `rclcpp`
- `nav2_costmap_2d`
- `nav_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`
- `tf2_geometry_msgs`
- `pluginlib`

## Installation

### Building from Source

1. **Create a workspace and clone the repository:**
```bash
mkdir -p ~/multi_robot_ws/src
cd ~/multi_robot_ws/src
git clone https://github.com/FilippoGuarda/multi_robot_costmap_plugin.git
```

2. **Install dependencies:**
```bash
cd ~/multi_robot_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the package:**
```bash
colcon build --packages-select multi_robot_costmap_plugin
source install/setup.bash
```

4. **Verify installation:**
```bash
ros2 pkg list | grep multi_robot_costmap_plugin
```

## Configuration

### GlobalCostmapFusion Node Configuration

Create a configuration file `config/global_costmap_fusion.yaml`:

```yaml
global_costmap_fusion:
  ros__parameters:
    # Robot fleet configuration
    robots: ["robot1", "robot2", "robot3", "robot4", "robot5", "robot6"]
    
    # Physical robot parameters
    robot_radius: 0.3              # Robot radius in meters
    exclusion_buffer: 0.5          # Additional buffer around robots
    
    # Update rates
    update_frequency: 10.0         # Hz - obstacle grid publishing rate
    robot_timeout: 5.0             # Seconds before marking robot inactive
    
    # Frame and topic configuration
    global_frame: "map"            # Global reference frame
    output_topic: "/shared_obstacles_grid"
    scan_topic: "scan"             # Laser scan topic suffix
    map_topic: "/map"              # Map topic for parameter extraction
    map_service: "/map_server/map" # Map service fallback
    base_frame_suffix: "base_footprint"  # Robot base frame suffix
```

### Nav2 Costmap Configuration

Add the MultiRobotLayer to your Nav2 costmap configuration:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "multi_robot_layer", "inflation_layer"]
      
      multi_robot_layer:
        plugin: "multi_robot_costmap_plugin/MultiRobotLayer"
        enabled: true
        robot_namespaces: ["robot1", "robot2", "robot3", "robot4", "robot5", "robot6"]
        obstacle_max_range: 2.5
        obstacle_min_range: 0.3
        footprint_padding: 0.1
        default_robot_radius: 0.3
```

## Usage

### Method 1: Integrated Launch File

```python
# Example launch file integration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('multi_robot_costmap_plugin'),
        'config',
        'global_costmap_fusion.yaml'
    )
    
    return LaunchDescription([
        # GlobalCostmapFusion node
        Node(
            package='multi_robot_costmap_plugin',
            executable='global_costmap_fusion_node',
            name='global_costmap_fusion',
            parameters=[config_file],
            output='screen'
        ),
        
        # Your Nav2 launch here...
    ])
```

### Method 2: Multi-Robot Fleet Launch

**Coming Soon** with complete simulation example

## Topics and Services

### Subscribed Topics (per robot)
- `/{robot_id}/scan` (sensor_msgs/LaserScan) - Laser scan data
- `/map` (nav_msgs/OccupancyGrid) - Map for parameter extraction

### Published Topics
- `/shared_obstacles_grid` (nav_msgs/OccupancyGrid) - Shared obstacle grid

### Required TF Frames (per robot)
- `map` → `{robot_id}/odom` → `{robot_id}/base_footprint`
- `{robot_id}/base_footprint` → `{robot_id}/laser_frame`

### Services Used
- `/map_server/map` (nav_msgs/GetMap) - Fallback for map parameters

## Architecture

### GlobalCostmapFusion Node

The fusion node operates with the following workflow:

1. **Initialization**: Subscribes to map topic/service to extract grid parameters
2. **Robot Tracking**: Uses TF2 to continuously track robot poses at 10Hz
3. **Scan Processing**: Processes laser scans from active robots
4. **Coordinate Transformation**: Converts scan points to global coordinates using TF2
5. **Robot Filtering**: Excludes points within robot exclusion zones
6. **Grid Publishing**: Publishes accumulated obstacles as an occupancy grid

### MultiRobotLayer Plugin

The costmap plugin integrates shared obstacle data:

1. **Lifecycle Management**: Properly initializes and activates with Nav2
2. **Data Subscription**: Subscribes to the shared obstacles grid
3. **Costmap Integration**: Adds shared obstacles to the robot's local costmap
4. **Thread Safety**: Ensures safe access to shared data structures

## Advanced Configuration

### Custom Robot Frame Configuration

```yaml
# For robots with different frame naming conventions
global_costmap_fusion:
  ros__parameters:
    base_frame_suffix: "base_link"  # Instead of "base_footprint"
```

### Performance Tuning

```yaml
# Optimize for large fleets
global_costmap_fusion:
  ros__parameters:
    update_frequency: 5.0          # Reduce for better performance
    robot_timeout: 10.0            # Increase for unstable networks
    exclusion_buffer: 0.3          # Reduce for denser operation
```

### Debug Configuration

```yaml
# Enable detailed logging
global_costmap_fusion:
  ros__parameters:
    # Add to launch with --log-level debug
```

## Troubleshooting

### Common Issues

**1. Plugin Not Loading**
```bash
# Check if plugin is registered
ros2 pkg list | grep multi_robot_costmap_plugin

# Verify plugin.xml is correct
cat install/multi_robot_costmap_plugin/share/multi_robot_costmap_plugin/plugin.xml
```

**2. No Shared Obstacles Published**
```bash
# Check if robots are active
ros2 topic echo /shared_obstacles_grid --once

# Verify laser scan topics
ros2 topic list | grep scan
ros2 topic hz /robot1/scan
```

**3. Transform Errors**
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Verify robot transforms
ros2 run tf2_ros tf2_echo map robot1/base_footprint
```

**4. Map Parameters Not Detected**
```bash
# Check map availability
ros2 topic echo /map --once
ros2 service call /map_server/map nav_msgs/srv/GetMap
```

### Performance Issues

- **High CPU usage**: Reduce `update_frequency` or `obstacle_max_range`
- **Memory consumption**: Limit the number of obstacles accumulated
- **Network bandwidth**: Reduce grid resolution or update frequency

### Debug Commands

```bash
# Monitor node performance
ros2 topic hz /shared_obstacles_grid

# Check robot activity
ros2 param get /global_costmap_fusion robots

# Monitor transform updates
ros2 topic echo /tf --once | grep robot1

# Check costmap plugin status
ros2 lifecycle get /global_costmap/global_costmap
```

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Citation

If you use this package in your research, please cite:

```bibtex
@software{multi_robot_costmap_plugin,
  author = {Filippo Guarda},
  title = {Multi-Robot Costmap Plugin for Nav2},
  year = {2025},
  url = {https://github.com/FilippoGuarda/multi_robot_costmap_plugin}
}
```

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Commit changes: `git commit -am 'Add feature'`
4. Push to branch: `git push origin feature-name`
5. Submit a pull request

## Support

For questions, issues, or contributions:
- **GitHub Issues**: [Report bugs or request features](https://github.com/FilippoGuarda/multi_robot_costmap_plugin/issues)
- **Discussions**: [Ask questions or share ideas](https://github.com/FilippoGuarda/multi_robot_costmap_plugin/discussions)

