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
  active_grid_(nullptr),
  roi_dirty_(false),
  grid_sequence_(0),
  last_processed_sequence_(0)
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

  // Declare standard parameters
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("shared_grid_topic", rclcpp::ParameterValue("/shared_obstacles"));
  declareParameter("robot_radius", rclcpp::ParameterValue(0.35));
  declareParameter("exclusion_buffer", rclcpp::ParameterValue(0.1));
  declareParameter("transform_timeout", rclcpp::ParameterValue(0.1));

  declareParameter("max_obstacles_per_update", rclcpp::ParameterValue(5000)); // Increased limit

  // Get parameters
  bool enabled;
  std::string shared_grid_topic;
  node->get_parameter(name_ + ".enabled", enabled);
  node->get_parameter(name_ + ".shared_grid_topic", shared_grid_topic);
  node->get_parameter(name_ + ".robot_radius", robot_radius_);
  node->get_parameter(name_ + ".exclusion_buffer", exclusion_buffer_);
  node->get_parameter(name_ + ".transform_timeout", transform_timeout_);
  node->get_parameter(name_ + ".max_obstacles_per_update", max_obstacles_per_update_);

  enabled_ = enabled;
  
  // Get robot namespace from node name
  std::string node_name = node->get_name();
  if (node_name.find('/') != std::string::npos) {
    robot_namespace_ = node_name.substr(1, node_name.find('/', 1) - 1);
  } else {
    robot_namespace_ = "robot1"; // fallback
  }

  // Set frame names
  global_frame_ = layered_costmap_->getGlobalFrameID();
  robot_base_frame_ = robot_namespace_ + "/base_link";

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize timestamps
  last_grid_update_ = node->get_clock()->now();

  // Subscribe to shared obstacle grid
  grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    shared_grid_topic,
    rclcpp::QoS(1).reliable().durability_volatile(),
    std::bind(&MultiRobotLayer::sharedGridCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), 
              "MultiRobotLayer initialized (NO THROTTLING) for robot %s, subscribing to %s", 
              robot_namespace_.c_str(), shared_grid_topic.c_str());

  // CRITICAL: Signal that this layer has current data
  current_ = true;
}

// Thread-safe grid access helper
nav_msgs::msg::OccupancyGrid::SharedPtr MultiRobotLayer::getActiveGrid()
{
  std::lock_guard<std::mutex> lock(active_grid_mutex_);
  return active_grid_;
}

void MultiRobotLayer::sharedGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

  auto now = node->get_clock()->now();

  // OPTIMIZATION 1: Use mutex-protected assignment
  {
    std::lock_guard<std::mutex> lock(active_grid_mutex_);
    active_grid_ = msg;
  }

  // OPTIMIZATION 2: Extract obstacles in background (non-blocking)
  extractObstaclesFromGrid(msg);

  grid_sequence_.fetch_add(1);
  last_grid_update_ = now;

  static int callback_count = 0;
  callback_count++;
  
  // Used for debugging, removed
  // if (callback_count % 10 == 1) { // More frequent logging since no throttling
  //   RCLCPP_INFO(node->get_logger(), 
  //               "Processed grid %d: %zu obstacles from shared topic (NO THROTTLING)", 
  //               callback_count, current_obstacles_.size());
  // }
}

void MultiRobotLayer::extractObstaclesFromGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
{
  if (!grid) return;

  // OPTIMIZATION 2: Build sparse obstacle representation
  std::unordered_set<ObstacleCell, ObstacleCellHash> new_obstacles;
  new_obstacles.reserve(max_obstacles_per_update_); // Pre-allocate

  int processed_count = 0;
  
  // Process grid efficiently - early exit if too many obstacles
  for (int y = 0; y < static_cast<int>(grid->info.height) && processed_count < max_obstacles_per_update_; ++y) {
    for (int x = 0; x < static_cast<int>(grid->info.width) && processed_count < max_obstacles_per_update_; ++x) {
      int index = y * grid->info.width + x;
      
      if (index < static_cast<int>(grid->data.size()) && grid->data[index] > 50) {
        new_obstacles.insert({x, y, static_cast<unsigned char>(LETHAL_OBSTACLE)});
        processed_count++;
      }
    }
  }

  // Update obstacle storage
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    previous_obstacles_ = std::move(current_obstacles_);
    current_obstacles_ = std::move(new_obstacles);
  }

  RCLCPP_DEBUG(node_.lock()->get_logger(), 
               "Extracted %zu obstacles from shared grid", current_obstacles_.size());
}

bool MultiRobotLayer::shouldUpdateCosts(double robot_x, double robot_y)
{
  (void)robot_x; // Mark as unused
  (void)robot_y; // Mark as unused
  return true;
}

void MultiRobotLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  (void)robot_yaw;  // Unused parameter
  
  if (!enabled_) {
    RCLCPP_DEBUG(node_.lock()->get_logger(), "MultiRobotLayer is DISABLED");
    return;
  }

  RCLCPP_DEBUG(node_.lock()->get_logger(), 
               "updateBounds called: robot at (%.2f, %.2f)", robot_x, robot_y);

  auto grid = getActiveGrid();
  if (!grid) {
    RCLCPP_DEBUG(node_.lock()->get_logger(), "No active grid available");
    return;
  }

  // Check if we have obstacles to process
  bool has_obstacles = false;
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    has_obstacles = !current_obstacles_.empty();
  }

  if (!has_obstacles) {
    RCLCPP_DEBUG(node_.lock()->get_logger(), "No obstacles to process");
    return;
  }

  double original_min_x = *min_x, original_max_x = *max_x;
  double original_min_y = *min_y, original_max_y = *max_y;

  // Expand bounds to include the entire shared grid area
  double grid_min_x = grid->info.origin.position.x;
  double grid_min_y = grid->info.origin.position.y;
  double grid_max_x = grid_min_x + grid->info.width * grid->info.resolution;
  double grid_max_y = grid_min_y + grid->info.height * grid->info.resolution;

  *min_x = std::min(*min_x, grid_min_x);
  *min_y = std::min(*min_y, grid_min_y);
  *max_x = std::max(*max_x, grid_max_x);
  *max_y = std::max(*max_y, grid_max_y);

  // Always include robot area for exclusion
  double robot_radius_with_buffer = robot_radius_ + exclusion_buffer_;
  *min_x = std::min(*min_x, robot_x - robot_radius_with_buffer);
  *min_y = std::min(*min_y, robot_y - robot_radius_with_buffer);
  *max_x = std::max(*max_x, robot_x + robot_radius_with_buffer);
  *max_y = std::max(*max_y, robot_y + robot_radius_with_buffer);

  // Log bounds changes for debugging
  if (*min_x != original_min_x || *max_x != original_max_x || 
      *min_y != original_min_y || *max_y != original_max_y) {
    RCLCPP_DEBUG(node_.lock()->get_logger(), 
                 "Bounds expanded from (%.2f,%.2f)-(%.2f,%.2f) to (%.2f,%.2f)-(%.2f,%.2f)",
                 original_min_x, original_min_y, original_max_x, original_max_y,
                 *min_x, *min_y, *max_x, *max_y);
  }

  need_recalculation_ = true;
}

void MultiRobotLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    RCLCPP_WARN(node_.lock()->get_logger(), "MultiRobotLayer disabled");
    return;
  }

  auto grid = getActiveGrid(); // OPTIMIZATION 1: Lock-free grid access
  if (!grid) {
    RCLCPP_WARN(node_.lock()->get_logger(), "No active grid available");
    return;
  }

  // Get robot position for exclusion
  double robot_x, robot_y;
  if (!getRobotPose(robot_x, robot_y)) {
    RCLCPP_WARN(node_.lock()->get_logger(), "Could not get robot pose");
    return;
  }

  int obstacles_applied = 0;
  
  // OPTIMIZATION 2: Process only sparse obstacle cells instead of full grid
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    RCLCPP_INFO(node_.lock()->get_logger(), 
                "Applying %zu obstacles to costmap bounds (%d,%d) to (%d,%d) - NO THROTTLING",
                current_obstacles_.size(), min_i, min_j, max_i, max_j);
    
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
          obstacles_applied++;
        }
      }
    }
  }

  RCLCPP_INFO(node_.lock()->get_logger(), "Applied %d obstacles to master costmap", obstacles_applied);

  // Mark as processed
  last_processed_sequence_ = grid_sequence_.load();
  need_recalculation_ = false;

  static int update_count = 0;
  update_count++;
  
  // Logging used for debugging
  // if (update_count % 5 == 1) { // More frequent logging since no throttling
  //   RCLCPP_INFO(node_.lock()->get_logger(), 
  //               "Cost update %d: processed %zu obstacles in bounds (%d,%d) to (%d,%d)",
  //               update_count, current_obstacles_.size(), min_i, min_j, max_i, max_j);
  // }
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
  return false;  // This layer adds persistent obstacles, so it's not clearable
}

void MultiRobotLayer::reset()
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  current_obstacles_.clear();
  previous_obstacles_.clear();
  need_recalculation_ = true;
}

void MultiRobotLayer::onFootprintChanged()
{
  need_recalculation_ = true;
}

} // namespace multi_robot_costmap_plugin
