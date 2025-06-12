# Multi-Robot Costmap Plugin - Build and Usage Instructions

## Package Structure

```
multi_robot_costmap_plugin/
├── CMakeLists.txt
├── package.xml
├── plugins.xml
├── include/
│   └── multi_robot_costmap_plugin/
│       └── footprint_aware_multi_robot_layer.hpp
├── src/
│   └── footprint_aware_multi_robot_layer.cpp
├── launch/
│   └── multi_robot_demo.launch.py
├── config/
│   └── nav2_params.yaml
├── test/
│   └── test_footprint_utils.cpp
└── README.md
```

## Building Package

### Option 1: Build as Standalone Package

1. **Create a new ROS2 workspace or navigate to existing one:**
   ```bash
   mkdir -p ~/multi_robot_ws/src
   cd ~/multi_robot_ws/src
   ```

2. **Clone or create package directory:**
   ```bash
   # If creating from scratch, create the directory and copy all files
   mkdir multi_robot_costmap_plugin
   cd multi_robot_costmap_plugin
   # Copy all generated files into this directory
   ```

3. **Build package:**
   ```bash
   cd ~/multi_robot_ws
   colcon build --packages-select multi_robot_costmap_plugin
   source install/setup.bash
   ```

4. **Run tests (optional):**
   ```bash
   colcon test --packages-select multi_robot_costmap_plugin
   colcon test-result --verbose
   ```

### Option 2: Add to Existing Navigation Package

If the plugin is to be integrated into an existing navigation package:

1. **Copy source files to an existing package:**
   ```bash
   # Assuming the package is called 'my_nav_package'
   cp include/multi_robot_costmap_plugin/* ~/pkg_workspace/src/my_nav_package/include/my_nav_package/
   cp src/* ~/pkg_workspace/src/my_nav_package/src/
   ```

2. **Update existing CMakeLists.txt:**
   Add dependencies and source file:
   ```cmake
   find_package(laser_geometry REQUIRED)
   # Add to dependencies list
   
   add_library(${PROJECT_NAME} SHARED
     # ... existing sources
     src/footprint_aware_multi_robot_layer.cpp
   )
   ```

3. **Update package.xml:**
   Add required dependencies from this package.xml

4. **Create or update plugins.xml in package:**
   Add plugin definition to existing plugins.xml

## Usage

### Method 1: Using Launch File

```bash
# Source workspace
source ~/multi_robot_ws/install/setup.bash

# Launch with default parameters
ros2 launch multi_robot_costmap_plugin multi_robot_demo.launch.py

# Launch with custom robot namespaces
ros2 launch multi_robot_costmap_plugin multi_robot_demo.launch.py \
  robot_namespaces:="['robot_a', 'robot_b', 'robot_c']"
```

### Method 2: Integrating with Existing Nav2 Configuration

1. **Add plugin to the existing Nav2 parameter file:**
   ```yaml
   local_costmap:
     local_costmap:
       ros__parameters:
         plugins: ["voxel_layer", "inflation_layer", "multi_robot_layer"]
         # ... other plugins
         multi_robot_layer:
           plugin: "multi_robot_costmap_plugin/FootprintAwareMultiRobotLayer"
           enabled: True
           robot_namespaces: ["robot1", "robot2", "robot3"]
           obstacle_max_range: 2.5
           obstacle_min_range: 0.0
           global_frame: "map"
           footprint_padding: 0.1
           default_robot_radius: 0.3
   ```

2. **Launch existing Nav2 stack:**
   ```bash
   ros2 launch the_navigation_package the_nav_launch.py
   ```

### Method 3: Manual Node Configuration

```bash
# Start Nav2 with custom parameters
ros2 run nav2_costmap_2d costmap_2d_node --ros-args \
  --params-file ~/multi_robot_ws/src/multi_robot_costmap_plugin/config/nav2_params.yaml
```

## Configuration Parameters

The plugin accepts the following parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_namespaces` | string[] | [] | List of robot namespaces to subscribe to |
| `obstacle_max_range` | double | 2.5 | Maximum range for obstacle detection (meters) |
| `obstacle_min_range` | double | 0.0 | Minimum range for obstacle detection (meters) |
| `global_frame` | string | "map" | Global reference frame |
| `footprint_padding` | double | 0.1 | Additional padding around robot footprints (meters) |
| `default_robot_radius` | double | 0.3 | Default robot radius when footprint is not available (meters) |

## Expected Topics

For each robot namespace (e.g., "robot1"), the plugin expects these topics:

- `/{robot_namespace}/scan` - sensor_msgs/LaserScan
- `/{robot_namespace}/amcl_pose` - geometry_msgs/PoseWithCovarianceStamped  
- `/{robot_namespace}/local_costmap/published_footprint` - geometry_msgs/PolygonStamped (optional)

## Troubleshooting

### Common Issues:

1. **Plugin not loading:**
   ```bash
   # Check if plugin is properly registered
   ros2 pkg list | grep multi_robot_costmap_plugin
   
   # Verify plugin registration
   ros2 run pluginlib pluginlib_headers nav2_costmap_2d
   ```

2. **Missing robot data:**
   ```bash
   # Check if robot topics are publishing
   ros2 topic list | grep robot1
   ros2 topic echo /robot1/scan --once
   ros2 topic echo /robot1/amcl_pose --once
   ```

3. **Transform issues:**
   ```bash
   # Check TF tree
   ros2 run tf2_tools view_frames.py
   
   # Verify transforms between robot frames
   ros2 run tf2_ros tf2_echo map robot1/base_link
   ```

4. **Performance issues:**
   - Reduce `obstacle_max_range` to limit processing
   - Decrease costmap update frequency
   - Reduce number of laser scan points processed

### Debug Information:

Enable debug logging to see plugin activity:
```bash
ros2 run nav2_costmap_2d costmap_2d_node --ros-args \
  --log-level multi_robot_costmap_plugin:=DEBUG
```

## Integration with Multi-Robot Task Allocation

This plugin is designed to work seamlessly with a multi robot task allocation system. It will:

1. **Share obstacle information** between all robots in the fleet
2. **Filter out robot detections** to prevent robots from avoiding each other unnecessarily  
3. **Maintain real-time performance** for dynamic replanning
4. **Provide safety margins** through configurable footprint padding

The plugin automatically integrates with Nav2's planning and control systems, so the existing task allocation logic should continue to work without modification while benefiting from improved multi-robot obstacle sharing.