// Copyright 2025 Filippo Guarda
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
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>
#include <map>
#include <vector>
#include <string>
#include <thread>

namespace multi_robot_costmap_plugin
{

class GlobalCostmapFusion : public rclcpp::Node
{
public:
  explicit GlobalCostmapFusion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~GlobalCostmapFusion();

private:
  struct RobotInfo {
    geometry_msgs::msg::TransformStamped transform; // Transform from global_frame to robot base_frame
    rclcpp::Time last_update;
    bool active;
    std::string base_frame; // Robot's base frame (e.g., "robot1/base_footprint")
  };

  void setupRobotInfo();
  void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string& robot_id);
  bool getRobotTransform(const std::string& robot_id, geometry_msgs::msg::TransformStamped& transform);
  bool isRobotLocation(const geometry_msgs::msg::Point& point);
  geometry_msgs::msg::Point convertScanPointToWorld(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    size_t point_index,
    const geometry_msgs::msg::TransformStamped& robot_transform);
  void publishSharedObstacles();
  void checkRobotActivity();
  void updateRobotTransforms();
  
  // Map-related functions
  void initializeFromMap();
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void requestMapFromService();

  // Parameters
  std::vector<std::string> robot_ids_;
  double robot_radius_;
  double exclusion_buffer_;
  double update_frequency_;
  double robot_timeout_;
  std::string global_frame_;
  std::string scan_topic_;
  std::string map_topic_;
  std::string map_service_;
  std::string base_frame_suffix_; // e.g., "base_footprint"
  
  // Map-derived parameters (no longer configurable)
  double grid_resolution_;
  double grid_width_;
  double grid_height_;
  geometry_msgs::msg::Pose grid_origin_;
  bool map_initialized_;

  // Publishers and subscribers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subs_;

  // Service client for map
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;

  // Robot tracking
  std::map<std::string, RobotInfo> robot_info_;
  std::mutex robot_info_mutex_;

  // Shared obstacle storage
  std::vector<geometry_msgs::msg::Point> shared_obstacles_;
  std::mutex obstacles_mutex_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timers
  rclcpp::TimerBase::SharedPtr activity_check_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr transform_update_timer_;
};

} // namespace multi_robot_costmap_plugin

#endif // MULTI_ROBOT_COSTMAP_PLUGIN__GLOBAL_COSTMAP_FUSION_HPP_
