cmake_minimum_required(VERSION 3.8)
project(multi_robot_costmap_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies including OpenCV for optimized memory operations
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED)  # Added for persistent memory optimization

include_directories(include ${OpenCV_INCLUDE_DIRS})

# Multi-robot layer plugin (unchanged)
add_library(${PROJECT_NAME} SHARED
  src/multi_robot_layer.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp nav2_costmap_2d geometry_msgs sensor_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs pluginlib
)

# Global costmap fusion with persistent memory
add_library(global_costmap_fusion_lib SHARED
  src/global_costmap_fusion.cpp
)
ament_target_dependencies(global_costmap_fusion_lib
  rclcpp geometry_msgs sensor_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs OpenCV
)

add_executable(global_costmap_fusion_node
  src/global_costmap_fusion_node.cpp
)
target_link_libraries(global_costmap_fusion_node
  global_costmap_fusion_lib
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(global_costmap_fusion_node rclcpp)

# Install targets
install(TARGETS ${PROJECT_NAME} global_costmap_fusion_lib global_costmap_fusion_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(nav2_costmap_2d plugin.xml)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} global_costmap_fusion_lib)
ament_export_dependencies(
  rclcpp nav2_costmap_2d geometry_msgs sensor_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs pluginlib OpenCV
)

ament_package()
