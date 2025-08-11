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
#include <algorithm>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(multi_robot_costmap_plugin::MultiRobotLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace multi_robot_costmap_plugin
{

MultiRobotLayer::MultiRobotLayer()
: need_recalculation_(false),
  last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max()),
  last_robot_x_(0.0),
  last_robot_y_(0.0),
  active_grid_(nullptr),  // ← FIXED: Regular shared_ptr instead of atomic
  roi_dirty_(false),
  grid_sequence_(0),
  last_processed_sequence_(0)
{
}

// Add this helper method for thread-safe grid access:
nav_msgs::msg::OccupancyGrid::SharedPtr MultiRobotLayer::getActiveGrid()
{
  std::lock_guard<std::mutex> lock(active_grid_mutex_);
  return active_grid_;
}

// Fix the callback to use mutex instead of atomic:
void MultiRobotLayer::sharedGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

  auto now = node->get_clock()->now();
  if ((now - last_grid_update_).seconds() < 0.05) { // Max 20Hz processing
    return;
  }

  // FIXED: Use mutex-protected assignment instead of atomic store
  {
    std::lock_guard<std::mutex> lock(active_grid_mutex_);
    active_grid_ = msg;
  }

  // Extract obstacles in background (non-blocking)
  extractObstaclesFromGrid(msg);

  grid_sequence_.fetch_add(1);
  last_grid_update_ = now;

  static int callback_count = 0;
  callback_count++;
  
  if (callback_count % 100 == 1) {
    RCLCPP_DEBUG(node->get_logger(), 
                 "Processed optimized grid %d: %zu obstacles", 
                 callback_count, current_obstacles_.size());
  }
}

// Fix updateBounds to use helper method:
void MultiRobotLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  (void)robot_yaw;  // Unused parameter
  
  if (!enabled_) {
    return;
  }

  if (!shouldUpdateCosts(robot_x, robot_y)) {
    return;
  }

  auto grid = getActiveGrid();  // ← FIXED: Use helper method instead of atomic load
  if (!grid) {
    return;
  }

  // Rest of the method remains the same...
}

// Fix updateCosts to use helper method:
void MultiRobotLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  auto grid = getActiveGrid();  // ← FIXED: Use helper method instead of atomic load
  if (!grid) {
    return;
  }

  // Get robot position for exclusion
  double robot_x, robot_y;
  if (!getRobotPose(robot_x, robot_y)) {
    return;
  }

  // ROI bounds
  if (roi_dirty_.load() && update_roi_.is_valid) {
    min_i = std::max(min_i, update_roi_.min_i);
    min_j = std::max(min_j, update_roi_.min_j);
    max_i = std::min(max_i, update_roi_.max_i);
    max_j = std::min(max_j, update_roi_.max_j);
  }

  // ROI only updates
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    for (const auto& obstacle : current_obstacles_) {
      // Check if obstacle is within update bounds
      if (obstacle.x >= min_i && obstacle.x < max_i &&
          obstacle.y >= min_j && obstacle.y < max_j) {
        
        // Convert to world coordinates for robot exclusion check
        double world_x, world_y;
        master_grid.mapToWorld(obstacle.x, obstacle.y, world_x, world_y);

        // Skip robot exclusion zone
        double dx = world_x - robot_x;
        double dy = world_y - robot_y;
        double distance = sqrt(dx*dx + dy*dy);
        
        if (distance >= robot_radius_ + exclusion_buffer_) {
          // OPTIMIZATION: Inlined cost application
          applyCostToCell(master_grid, obstacle.x, obstacle.y, obstacle.cost);
        }
      }
    }
  }

  // Mark as processed
  last_processed_sequence_ = grid_sequence_.load();
  roi_dirty_.store(false);
  need_recalculation_ = false;

  static int update_count = 0;
  update_count++;
  
  if (update_count % 50 == 1) {
    RCLCPP_DEBUG(node_.lock()->get_logger(), 
                 "Optimized cost update %d: processed %zu obstacles in bounds (%d,%d) to (%d,%d)",
                 update_count, current_obstacles_.size(), min_i, min_j, max_i, max_j);
  }
}

inline void MultiRobotLayer::applyCostToCell(nav2_costmap_2d::Costmap2D & master_grid, int i, int j, unsigned char cost)
{
  unsigned char current_cost = master_grid.getCost(i, j);
  if (cost > current_cost) {
    master_grid.setCost(i, j, cost);
  }
}

bool MultiRobotLayer::getRobotPose(double& x, double& y)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(transform_timeout_));
    
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    return true;
  } catch (const tf2::TransformException& ex) {
    return false;
  }
}

bool MultiRobotLayer::isClearable()
{
  return false;  // This layer adds persistent obstacles
}

void MultiRobotLayer::reset()
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  current_obstacles_.clear();
  previous_obstacles_.clear();
  update_roi_ = ROI();
  roi_dirty_.store(false);
  need_recalculation_ = true;
}

void MultiRobotLayer::onFootprintChanged()
{
  need_recalculation_ = true;
}

} // namespace multi_robot_costmap_plugin
