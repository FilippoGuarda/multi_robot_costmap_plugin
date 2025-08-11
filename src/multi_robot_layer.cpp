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

PLUGINLIB_EXPORT_CLASS(multi_robot_costmap_plugin::MultiRobotLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

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

  // Declare and get parameters
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("shared_grid_topic", rclcpp::ParameterValue("/shared_obstacles"));
  declareParameter("robot_radius", rclcpp::ParameterValue(0.35));
  declareParameter("exclusion_buffer", rclcpp::ParameterValue(0.1));
  declareParameter("transform_timeout", rclcpp::ParameterValue(0.1));

  bool enabled;
  std::string shared_grid_topic;
  node->get_parameter(name_ + ".enabled", enabled);
  node->get_parameter(name_ + ".shared_grid_topic", shared_grid_topic);
  node->get_parameter(name_ + ".robot_radius", robot_radius_);
  node->get_parameter(name_ + ".exclusion_buffer", exclusion_buffer_);
  node->get_parameter(name_ + ".transform_timeout", transform_timeout_);

  enabled_ = enabled;
  
  // Get robot namespace from tf_prefix or node namespace
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

  // Subscribe to shared obstacle grid
  grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    shared_grid_topic,
    rclcpp::QoS(1).reliable(),
    std::bind(&MultiRobotLayer::sharedGridCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), 
              "MultiRobotLayer initialized for robot %s, subscribing to %s", 
              robot_namespace_.c_str(), shared_grid_topic.c_str());

  current_ = true;
}

void MultiRobotLayer::sharedGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(grid_mutex_);
  latest_shared_grid_ = msg;
  need_recalculation_ = true;
  
  static int callback_count = 0;
  callback_count++;
  
  if (callback_count % 50 == 1) {
    RCLCPP_DEBUG(node_.lock()->get_logger(), 
                 "Received shared grid %d: %dx%d", 
                 callback_count, msg->info.width, msg->info.height);
  }
}

void MultiRobotLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  (void)robot_yaw;  // Mark as intentionally unused
  
  if (!enabled_ || !latest_shared_grid_) {
    return;
  }

  std::lock_guard<std::mutex> lock(grid_mutex_);
  
  // If we have new data or need recalculation, expand bounds to cover entire shared grid
  if (need_recalculation_ || latest_shared_grid_) {
    double grid_min_x = latest_shared_grid_->info.origin.position.x;
    double grid_min_y = latest_shared_grid_->info.origin.position.y;
    double grid_max_x = grid_min_x + latest_shared_grid_->info.width * latest_shared_grid_->info.resolution;
    double grid_max_y = grid_min_y + latest_shared_grid_->info.height * latest_shared_grid_->info.resolution;

    // Expand bounds to include the shared grid area
    *min_x = std::min(*min_x, grid_min_x);
    *min_y = std::min(*min_y, grid_min_y);
    *max_x = std::max(*max_x, grid_max_x);
    *max_y = std::max(*max_y, grid_max_y);

    // Also include robot position
    double robot_radius_with_buffer = robot_radius_ + exclusion_buffer_;
    *min_x = std::min(*min_x, robot_x - robot_radius_with_buffer);
    *min_y = std::min(*min_y, robot_y - robot_radius_with_buffer);
    *max_x = std::max(*max_x, robot_x + robot_radius_with_buffer);
    *max_y = std::max(*max_y, robot_y + robot_radius_with_buffer);

    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    
    need_recalculation_ = true;
  }
}

void MultiRobotLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !latest_shared_grid_) {
    return;
  }

  std::lock_guard<std::mutex> lock(grid_mutex_);

  auto node = node_.lock();
  if (!node) {
    return;
  }

  // Get robot position for exclusion
  double robot_x, robot_y;
  if (!getRobotPose(robot_x, robot_y)) {
    RCLCPP_DEBUG(node->get_logger(), "Could not get robot pose for cost update");
    return;
  }

  static int update_count = 0;
  update_count++;
  
  if (update_count % 100 == 1) {
    RCLCPP_DEBUG(node->get_logger(), 
                 "Updating costs %d: bounds=(%d,%d) to (%d,%d), robot at (%.2f,%.2f)",
                 update_count, min_i, min_j, max_i, max_j, robot_x, robot_y);
  }

  // Apply shared obstacles to the master costmap
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      // Convert costmap coordinates to world coordinates
      double world_x, world_y;
      master_grid.mapToWorld(i, j, world_x, world_y);

      // Skip if this is near the robot's location
      double dx = world_x - robot_x;
      double dy = world_y - robot_y;
      double distance = sqrt(dx*dx + dy*dy);
      
      if (distance < robot_radius_ + exclusion_buffer_) {
        continue; // Skip robot's exclusion zone
      }

      // Convert world coordinates to shared grid coordinates
      int shared_i = static_cast<int>((world_x - latest_shared_grid_->info.origin.position.x) / 
                                      latest_shared_grid_->info.resolution);
      int shared_j = static_cast<int>((world_y - latest_shared_grid_->info.origin.position.y) / 
                                      latest_shared_grid_->info.resolution);

      // Check bounds in shared grid
      if (shared_i >= 0 && shared_i < static_cast<int>(latest_shared_grid_->info.width) &&
          shared_j >= 0 && shared_j < static_cast<int>(latest_shared_grid_->info.height)) {
        
        int shared_index = shared_j * latest_shared_grid_->info.width + shared_i;
        
        if (shared_index >= 0 && shared_index < static_cast<int>(latest_shared_grid_->data.size())) {
          int8_t shared_value = latest_shared_grid_->data[shared_index];
          
          // Apply obstacle if it exists in shared grid
          if (shared_value > 50) { // Occupied in shared grid
            unsigned char cost = master_grid.getCost(i, j);
            master_grid.setCost(i, j, std::max(cost, static_cast<unsigned char>(LETHAL_OBSTACLE)));
          }
        }
      }
    }
  }

  need_recalculation_ = false;
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
  std::lock_guard<std::mutex> lock(grid_mutex_);
  need_recalculation_ = true;
}

void MultiRobotLayer::onFootprintChanged()
{
  need_recalculation_ = true;
}

} // namespace multi_robot_costmap_plugin
