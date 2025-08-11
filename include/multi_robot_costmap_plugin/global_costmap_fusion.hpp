// Copyright 2023 Filippo Guarda
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MULTI_ROBOT_COSTMAP_PLUGIN__GLOBAL_COSTMAP_FUSION_HPP_
#define MULTI_ROBOT_COSTMAP_PLUGIN__GLOBAL_COSTMAP_FUSION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// OpenCV for optimized matrix operations and ray tracing
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <mutex>

namespace multi_robot_costmap_plugin
{

class GlobalCostmapFusion : public rclcpp::Node
{
public:
  explicit GlobalCostmapFusion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~GlobalCostmapFusion();

private:
  // Robot information structure
  struct RobotInfo {
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Time last_update;
    bool active;
    std::string base_frame;
  };

  // Persistent obstacle memory with OpenCV matrices
  struct ObstacleMemory {
    cv::Mat obstacle_grid;          // Persistent obstacle occupancy (CV_8UC1: 0=free, 100=occupied)
    cv::Mat timestamp_grid;         // Last observation time per cell (CV_64FC1)
    cv::Mat confidence_grid;        // Detection confidence/count per cell (CV_32FC1)
    std::mutex memory_mutex;        // Thread safety for memory access
    bool initialized;
    
    ObstacleMemory() : initialized(false) {}
  };

  // Core lifecycle functions
  void setupRobotInfo();
  void initializeFromMap();
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void requestMapFromService();

  // Laser scan processing
  void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string& robot_id);
  bool getRobotTransform(const std::string& robot_id, geometry_msgs::msg::TransformStamped& transform);
  geometry_msgs::msg::Point convertScanPointToWorld(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    size_t point_index,
    const geometry_msgs::msg::TransformStamped& robot_transform);
  bool isRobotLocation(const geometry_msgs::msg::Point& point);

  // Enhanced persistent memory and ray tracing
  void initializeObstacleMemory();
  void updateObstacleMemory(const std::vector<geometry_msgs::msg::Point>& obstacles, 
                            const geometry_msgs::msg::Point& robot_pos);
  void rayTraceClearance(const geometry_msgs::msg::Point& robot_pos, 
                         const std::vector<geometry_msgs::msg::Point>& endpoints);
  void rayTraceLine(const cv::Point2i& start, const cv::Point2i& end, 
                    std::vector<cv::Point2i>& line_points) const;
  
  // Coordinate transformations
  cv::Point2i worldToGrid(const geometry_msgs::msg::Point& world_point) const;
  geometry_msgs::msg::Point gridToWorld(const cv::Point2i& grid_point) const;

  // Periodic maintenance
  void updateRobotTransforms();
  void checkRobotActivity();
  void cleanupOldObstacles();
  void publishSharedObstacles();
  void markRobotFootprint(nav_msgs::msg::OccupancyGrid& grid, double robot_x, double robot_y);

  // Configuration parameters
  std::vector<std::string> robot_ids_;
  double robot_radius_;
  double exclusion_buffer_;
  double update_frequency_;
  double robot_timeout_;
  std::string global_frame_;
  std::string scan_topic_;
  std::string map_topic_;
  std::string map_service_;
  std::string base_frame_suffix_;
  
  // Memory management parameters
  double obstacle_decay_time_;        // Time before obstacles start decaying
  double max_obstacle_age_;           // Maximum age before removal
  int min_detection_count_;           // Minimum hits to mark as obstacle
  bool enable_memory_cleanup_;        // Enable automatic cleanup

  // Map-derived parameters (set from received map)
  double grid_resolution_;
  double grid_width_;
  double grid_height_;
  geometry_msgs::msg::Pose grid_origin_;
  bool map_initialized_;

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subs_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;

  // Robot tracking with thread safety
  std::map<std::string, RobotInfo> robot_info_;
  std::mutex robot_info_mutex_;

  // Enhanced persistent obstacle memory system
  ObstacleMemory obstacle_memory_;

  // TF handling
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timers for periodic operations
  rclcpp::TimerBase::SharedPtr transform_update_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr activity_check_timer_;
  rclcpp::TimerBase::SharedPtr memory_cleanup_timer_;
};

} // namespace multi_robot_costmap_plugin

#endif // MULTI_ROBOT_COSTMAP_PLUGIN__GLOBAL_COSTMAP_FUSION_HPP_
