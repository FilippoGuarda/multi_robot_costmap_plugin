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
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include <tf2/utils.h>
#include <cmath>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

PLUGINLIB_EXPORT_CLASS(multi_robot_costmap_plugin::MultiRobotLayer, nav2_costmap_2d::Layer)

namespace multi_robot_costmap_plugin
{

MultiRobotLayer::MultiRobotLayer()
: need_recalculation_(false),
  last_min_x_(-std::numeric_limits<double>::max()),
  last_min_y_(-std::numeric_limits<double>::max()),
  last_max_x_(std::numeric_limits<double>::max()),
  last_max_y_(std::numeric_limits<double>::max())
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

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("shared_grid_topic", rclcpp::ParameterValue("/shared_obstacles_grid"));
  declareParameter("robot_namespace", rclcpp::ParameterValue("robot1"));
  declareParameter("robot_radius", rclcpp::ParameterValue(0.3));
  declareParameter("exclusion_buffer", rclcpp::ParameterValue(0.5));
  declareParameter("global_frame", rclcpp::ParameterValue("map"));
  declareParameter("robot_base_frame", rclcpp::ParameterValue("base_footprint"));
  declareParameter("transform_timeout", rclcpp::ParameterValue(0.1));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "shared_grid_topic", shared_grid_topic_);
  node->get_parameter(name_ + "." + "robot_namespace", robot_namespace_);
  node->get_parameter(name_ + "." + "robot_radius", robot_radius_);
  node->get_parameter(name_ + "." + "exclusion_buffer", exclusion_buffer_);
  node->get_parameter(name_ + "." + "global_frame", global_frame_);
  node->get_parameter(name_ + "." + "robot_base_frame", robot_base_frame_);
  node->get_parameter(name_ + "." + "transform_timeout", transform_timeout_);

  if (!robot_namespace_.empty()) {
    robot_base_frame_ = robot_namespace_ + "/" + robot_base_frame_;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribe to shared obstacles grid
  grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    shared_grid_topic_,
    rclcpp::QoS(10).reliable(),
    std::bind(&MultiRobotLayer::sharedGridCallback, this, std::placeholders::_1));

  need_recalculation_ = false;
  current_ = true;

  RCLCPP_INFO(
    node->get_logger(),
    "MultiRobotLayer initialized for robot: %s, frames: %s -> %s, listening on topic: %s",
    robot_namespace_.c_str(), global_frame_.c_str(), robot_base_frame_.c_str(), 
    shared_grid_topic_.c_str());
}

bool MultiRobotLayer::getRobotPose(double& x, double& y, double& yaw)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      global_frame_,
      robot_base_frame_,
      rclcpp::Time(0),
      rclcpp::Duration::from_seconds(transform_timeout_)
    );

    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    yaw = tf2::getYaw(transform.transform.rotation);
    
    return true;
  } catch (const tf2::TransformException& ex) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_DEBUG(node->get_logger(), 
                  "Could not get robot pose: %s", ex.what());
    }
    return false;
  }
}

void MultiRobotLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;



  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = -std::numeric_limits<double>::max();
    *min_y = -std::numeric_limits<double>::max();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();

    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;

    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

void MultiRobotLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(grid_mutex_);
  if (!latest_shared_grid_) {
    return;
  }

  auto node = node_.lock();
  if (!node) return;

  // Get current robot pose
  double robot_x, robot_y, robot_yaw;
  if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
    RCLCPP_DEBUG(node->get_logger(), 
                "Failed to get robot pose, skipping obstacle update");
    return;
  }

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(master_grid.getSizeInCellsX()), max_i);
  max_j = std::min(static_cast<int>(master_grid.getSizeInCellsY()), max_j);


  const auto& shared_grid = latest_shared_grid_;
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      double world_x, world_y;
      master_grid.mapToWorld(i, j, world_x, world_y);

      // Convert world coordinates to shared grid coordinates
      int shared_x = static_cast<int>(
        (world_x - shared_grid->info.origin.position.x) / shared_grid->info.resolution);
      int shared_y = static_cast<int>(
        (world_y - shared_grid->info.origin.position.y) / shared_grid->info.resolution);

      // Check bounds in shared grid
      if (shared_x >= 0 && shared_x < static_cast<int>(shared_grid->info.width) &&
          shared_y >= 0 && shared_y < static_cast<int>(shared_grid->info.height)) {
        
        int shared_index = shared_y * shared_grid->info.width + shared_x;

        // If there's an obstacle in the shared grid and it's not too close to current robot
        if (shared_grid->data[shared_index] > 50 && 
            shouldApplyObstacle(world_x, world_y, robot_x, robot_y)) {
          master_grid.setCost(i, j, LETHAL_OBSTACLE);
        }
      }
    }
  }
}

void MultiRobotLayer::reset()
{
  std::lock_guard<std::mutex> lock(grid_mutex_);
  latest_shared_grid_.reset();
  need_recalculation_ = true;
}

void MultiRobotLayer::onFootprintChanged()
{
  need_recalculation_ = true;
}

void MultiRobotLayer::sharedGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(grid_mutex_);
  latest_shared_grid_ = msg;
  need_recalculation_ = true;
}

bool MultiRobotLayer::shouldApplyObstacle(double obs_x, double obs_y, double robot_x, double robot_y)
{
  double dx = obs_x - robot_x;
  double dy = obs_y - robot_y;
  double distance = sqrt(dx*dx + dy*dy);
  return distance > (robot_radius_ + exclusion_buffer_);
}

} // namespace multi_robot_costmap_plugin
