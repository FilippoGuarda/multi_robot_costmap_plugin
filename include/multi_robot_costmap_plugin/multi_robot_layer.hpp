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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/msg/costmap_update.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>
#include <vector>
#include <string>

namespace multi_robot_costmap_plugin
{

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

private:
  struct CostmapCell {
    double x, y;
    unsigned char cost;
    rclcpp::Time timestamp;
  };

  void costmapUpdateCallback(const nav2_msgs::msg::CostmapUpdate::SharedPtr msg);
  bool shouldApplyUpdate(const CostmapCell& cell, double robot_x, double robot_y);
  void clearOldUpdates();

  rclcpp::Subscription<nav2_msgs::msg::CostmapUpdate>::SharedPtr update_sub_;
  
  std::vector<CostmapCell> pending_updates_;
  std::mutex update_mutex_;
  
  std::string robot_namespace_;
  std::string update_topic_;
  double robot_radius_;
  double exclusion_buffer_;
  double max_update_age_;
  
  rclcpp::Time last_update_time_;
  bool need_recalculation_;
  
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
};

}  // namespace multi_robot_costmap_plugin

#endif  // MULTI_ROBOT_COSTMAP_PLUGIN__MULTI_ROBOT_LAYER_HPP_