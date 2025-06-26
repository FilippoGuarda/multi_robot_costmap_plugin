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

#include "multi_robot_costmap_plugin/multi_robot_layer.hpp"
#include <nav2_costmap_2d/costmap_math.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(multi_robot_costmap_plugin::MultiRobotLayer, nav2_costmap_2d::Layer)

namespace multi_robot_costmap_plugin
{

MultiRobotLayer::MultiRobotLayer()
: need_recalculation_(false),
  last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

MultiRobotLayer::~MultiRobotLayer()
{
}

void MultiRobotLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  declareParameter("update_topic", rclcpp::ParameterValue("/shared_costmap_updates"));
  declareParameter("robot_namespace", rclcpp::ParameterValue("robot1"));
  declareParameter("robot_radius", rclcpp::ParameterValue(0.3));
  declareParameter("exclusion_buffer", rclcpp::ParameterValue(0.5));
  declareParameter("max_update_age", rclcpp::ParameterValue(5.0));

  node->get_parameter(name_ + "." + "update_topic", update_topic_);
  node->get_parameter(name_ + "." + "robot_namespace", robot_namespace_);
  node->get_parameter(name_ + "." + "robot_radius", robot_radius_);
  node->get_parameter(name_ + "." + "exclusion_buffer", exclusion_buffer_);
  node->get_parameter(name_ + "." + "max_update_age", max_update_age_);

  // Subscribe to shared costmap updates
  update_sub_ = node->create_subscription<nav2_msgs::msg::CostmapUpdate>(
    update_topic_,
    rclcpp::QoS(10).reliable(),
    std::bind(&MultiRobotLayer::costmapUpdateCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(),
    "MultiRobotLayer initialized for robot: %s, listening on topic: %s",
    robot_namespace_.c_str(), update_topic_.c_str());

  current_ = true;
  enabled_ = true;
}

void MultiRobotLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;

  clearOldUpdates();

  std::lock_guard<std::mutex> lock(update_mutex_);

  if (pending_updates_.empty() && !need_recalculation_) {
    return;
  }

  if (need_recalculation_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
  }

  for (const auto& cell : pending_updates_) {
    if (shouldApplyUpdate(cell, robot_x, robot_y)) {
      *min_x = std::min(*min_x, cell.x - robot_radius_);
      *min_y = std::min(*min_y, cell.y - robot_radius_);
      *max_x = std::max(*max_x, cell.x + robot_radius_);
      *max_y = std::max(*max_y, cell.y + robot_radius_);
    }
  }

  last_min_x_ = *min_x;
  last_min_y_ = *min_y;
  last_max_x_ = *max_x;
  last_max_y_ = *max_y;
}

void MultiRobotLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(update_mutex_);

  auto node = node_.lock();
  if (!node) return;

  // Get current robot position
  double robot_x = 0.0, robot_y = 0.0;
  if (layered_costmap_->getRobotPose(robot_x, robot_y)) {
    
    // Apply pending updates
    for (const auto& cell : pending_updates_) {
      if (shouldApplyUpdate(cell, robot_x, robot_y)) {
        unsigned int mx, my;
        if (master_grid.worldToMap(cell.x, cell.y, mx, my)) {
          if (mx >= static_cast<unsigned int>(min_i) && mx <= static_cast<unsigned int>(max_i) &&
              my >= static_cast<unsigned int>(min_j) && my <= static_cast<unsigned int>(max_j)) {
            master_grid.setCost(mx, my, cell.cost);
          }
        }
      }
    }
  }

  pending_updates_.clear();
  need_recalculation_ = false;
  last_update_time_ = node->now();
}

void MultiRobotLayer::reset()
{
  std::lock_guard<std::mutex> lock(update_mutex_);
  pending_updates_.clear();
  need_recalculation_ = true;
}

void MultiRobotLayer::onFootprintChanged()
{
  need_recalculation_ = true;
}

void MultiRobotLayer::costmapUpdateCallback(const nav2_msgs::msg::CostmapUpdate::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(update_mutex_);
  
  auto node = node_.lock();
  if (!node) return;

  // Convert and store updates
  for (const auto& cell : msg->cells) {
    CostmapCell new_cell;
    new_cell.x = cell.x;
    new_cell.y = cell.y;
    new_cell.cost = cell.cost;
    new_cell.timestamp = node->now();
    pending_updates_.push_back(new_cell);
  }

  need_recalculation_ = true;
}

bool MultiRobotLayer::shouldApplyUpdate(const CostmapCell& cell, double robot_x, double robot_y)
{
  // Check if update is too close to current robot
  double dx = cell.x - robot_x;
  double dy = cell.y - robot_y;
  double distance = sqrt(dx*dx + dy*dy);
  
  return distance > (robot_radius_ + exclusion_buffer_);
}

void MultiRobotLayer::clearOldUpdates()
{
  auto node = node_.lock();
  if (!node) return;

  std::lock_guard<std::mutex> lock(update_mutex_);
  
  auto current_time = node->now();
  auto max_age = rclcpp::Duration::from_seconds(max_update_age_);
  
  pending_updates_.erase(
    std::remove_if(pending_updates_.begin(), pending_updates_.end(),
      [current_time, max_age](const CostmapCell& cell) {
        return (current_time - cell.timestamp) > max_age;
      }),
    pending_updates_.end());
}

}  // namespace multi_robot_costmap_plugin