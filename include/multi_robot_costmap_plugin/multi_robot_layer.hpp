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

#ifndef MULTI_ROBOT_COSTMAP_PLUGIN__MULTI_ROBOT_LAYER_HPP_
#define MULTI_ROBOT_COSTMAP_PLUGIN__MULTI_ROBOT_LAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <mutex>
#include <atomic>
#include <string>
#include <memory>
#include <unordered_set>
#include <unordered_map>

namespace multi_robot_costmap_plugin
{

// Sparse obstacle cell representation
struct ObstacleCell {
  int x, y;
  unsigned char cost;
  
  bool operator==(const ObstacleCell& other) const {
    return x == other.x && y == other.y;
  }
};

// Hash function for ObstacleCell
struct ObstacleCellHash {
  std::size_t operator()(const ObstacleCell& cell) const {
    return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
  }
};

// Region of Interest bounds
struct ROI {
  int min_i, min_j, max_i, max_j;
  bool is_valid;
  
  ROI() : min_i(INT_MAX), min_j(INT_MAX), max_i(INT_MIN), max_j(INT_MIN), is_valid(false) {}
  
  void expand(int i, int j) {
    if (!is_valid) {
      min_i = max_i = i;
      min_j = max_j = j;
      is_valid = true;
    } else {
      min_i = std::min(min_i, i);
      min_j = std::min(min_j, j);
      max_i = std::max(max_i, i);
      max_j = std::max(max_j, j);
    }
  }
};

class MultiRobotLayer : public nav2_costmap_2d::Layer
{
public:
  MultiRobotLayer();
  virtual ~MultiRobotLayer();

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  virtual void reset() override;
  virtual void onFootprintChanged() override;
  virtual bool isClearable() override;

private:
  // Configuration parameters
  std::string robot_namespace_;
  std::string shared_grid_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double robot_radius_;
  double exclusion_buffer_;
  double transform_timeout_;

  // Optimization parameters
  double min_update_distance_;
  double roi_padding_;
  int max_obstacles_per_update_;

  // Update tracking
  bool need_recalculation_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  double last_robot_x_, last_robot_y_;

  // ROS interfaces  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // FIXED: Use mutex-protected shared_ptr instead of atomic (C++11 compatible)
  nav_msgs::msg::OccupancyGrid::SharedPtr active_grid_;
  std::mutex active_grid_mutex_;

  // OPTIMIZATION 2: Sparse obstacle storage
  std::unordered_set<ObstacleCell, ObstacleCellHash> current_obstacles_;
  std::unordered_set<ObstacleCell, ObstacleCellHash> previous_obstacles_;
  std::mutex obstacles_mutex_;

  // OPTIMIZATION 3: Region-of-Interest tracking
  ROI update_roi_;
  std::atomic<bool> roi_dirty_;

  // OPTIMIZATION 4: Throttled updates
  rclcpp::Time last_grid_update_;
  std::atomic<uint64_t> grid_sequence_;
  uint64_t last_processed_sequence_;

  // Private methods
  void sharedGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  bool getRobotPose(double& x, double& y);
  void extractObstaclesFromGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr grid);
  void updateROIFromChanges();
  bool shouldUpdateCosts(double robot_x, double robot_y);
  inline void applyCostToCell(nav2_costmap_2d::Costmap2D & master_grid, int i, int j, unsigned char cost);
  nav_msgs::msg::OccupancyGrid::SharedPtr getActiveGrid();
};

} // namespace multi_robot_costmap_plugin

#endif // MULTI_ROBOT_COSTMAP_PLUGIN__MULTI_ROBOT_LAYER_HPP_
