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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <mutex>
#include <string>
#include <memory>

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
  virtual bool isClearable() override { return false; }

private:
  // Robot pose storage
  double robot_x_;
  double robot_y_;
  double robot_yaw_;

  void sharedGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  bool shouldApplyObstacle(double obs_x, double obs_y, double robot_x, double robot_y);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_shared_grid_;
  std::mutex grid_mutex_;

  std::string robot_namespace_;
  std::string shared_grid_topic_;
  double robot_radius_;
  double exclusion_buffer_;

  rclcpp::Time last_update_time_;
  bool need_recalculation_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
};

} // namespace multi_robot_costmap_plugin

#endif // MULTI_ROBOT_COSTMAP_PLUGIN__MULTI_ROBOT_LAYER_HPP_