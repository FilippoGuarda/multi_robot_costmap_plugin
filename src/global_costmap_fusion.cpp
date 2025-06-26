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

#include "multi_robot_costmap_plugin/global_costmap_fusion.hpp"
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace multi_robot_costmap_plugin
{

GlobalCostmapFusion::GlobalCostmapFusion(const rclcpp::NodeOptions & options)
: Node("global_costmap_fusion", options)
{
  // Declare parameters
  this->declare_parameter("robots", std::vector<std::string>{"robot1"});
  this->declare_parameter("robot_radius", 0.3);
  this->declare_parameter("exclusion_buffer", 0.5);
  this->declare_parameter("update_frequency", 10.0);
  this->declare_parameter("robot_timeout", 5.0);
  this->declare_parameter("global_frame", "map");
  this->declare_parameter("output_topic", "/shared_costmap_updates");

  // Get parameters
  robot_ids_ = this->get_parameter("robots").as_string_array();
  robot_radius_ = this->get_parameter("robot_radius").as_double();
  exclusion_buffer_ = this->get_parameter("exclusion_buffer").as_double();
  update_frequency_ = this->get_parameter("update_frequency").as_double();
  robot_timeout_ = this->get_parameter("robot_timeout").as_double();
  global_frame_ = this->get_parameter("global_frame").as_string();

  std::string output_topic = this->get_parameter("output_topic").as_string();

  // Setup TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create publisher
  update_pub_ = this->create_publisher<nav2_msgs::msg::CostmapUpdate>(
    output_topic, rclcpp::QoS(10).reliable());

  // Setup subscriptions for each robot
  setupSubscriptions();

  // Timer for checking robot activity
  activity_check_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&GlobalCostmapFusion::checkRobotActivity, this));

  RCLCPP_INFO(this->get_logger(), "GlobalCostmapFusion initialized for %zu robots", robot_ids_.size());
}

GlobalCostmapFusion::~GlobalCostmapFusion()
{
}

void GlobalCostmapFusion::setupSubscriptions()
{
  for (const auto& robot_id : robot_ids_) {
    // Create scan subscription
    std::string scan_topic = "/" + robot_id + "/scan";
    auto scan_callback = [this, robot_id](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      this->processLaserScan(msg, robot_id);
    };
    
    scan_subs_[robot_id] = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(), scan_callback);

    // Create pose subscription
    std::string pose_topic = "/" + robot_id + "/pose";
    auto pose_callback = [this, robot_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      this->updateRobotPose(msg, robot_id);
    };
    
    pose_subs_[robot_id] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, rclcpp::QoS(10), pose_callback);

    // Initialize robot info
    robot_info_[robot_id] = RobotInfo{};
    robot_info_[robot_id].active = false;

    RCLCPP_INFO(this->get_logger(), "Setup subscriptions for robot: %s", robot_id.c_str());
  }
}

void GlobalCostmapFusion::processLaserScan(
  const sensor_msgs::msg::LaserScan::SharedPtr msg,
  const std::string& robot_id)
{
  std::lock_guard<std::mutex> lock(robot_info_mutex_);

  // Check if we have pose information for this robot
  if (robot_info_.find(robot_id) == robot_info_.end() || !robot_info_[robot_id].active) {
    return;
  }

  const auto& robot_pose = robot_info_[robot_id].pose;
  std::vector<nav2_msgs::msg::CostmapCell> updates;

  // Process each laser scan point
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max) {
      continue;
    }

    // Convert scan point to world coordinates
    geometry_msgs::msg::Point world_point = convertScanPointToWorld(msg, i, robot_pose);

    // Skip if this point corresponds to another robot's location
    if (isRobotLocation(world_point)) {
      continue;
    }

    // Create costmap cell update
    nav2_msgs::msg::CostmapCell cell;
    cell.x = world_point.x;
    cell.y = world_point.y;
    cell.cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    updates.push_back(cell);
  }

  // Publish updates if any
  if (!updates.empty()) {
    publishCostmapUpdate(updates);
  }
}

void GlobalCostmapFusion::updateRobotPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg,
  const std::string& robot_id)
{
  std::lock_guard<std::mutex> lock(robot_info_mutex_);
  
  robot_info_[robot_id].pose = *msg;
  robot_info_[robot_id].last_update = this->now();
  robot_info_[robot_id].active = true;
}

bool GlobalCostmapFusion::isRobotLocation(const geometry_msgs::msg::Point& point)
{
  for (const auto& [robot_id, info] : robot_info_) {
    if (!info.active) continue;

    double dx = point.x - info.pose.pose.position.x;
    double dy = point.y - info.pose.pose.position.y;
    double distance = sqrt(dx*dx + dy*dy);

    if (distance < (robot_radius_ + exclusion_buffer_)) {
      return true;
    }
  }
  return false;
}

geometry_msgs::msg::Point GlobalCostmapFusion::convertScanPointToWorld(
  const sensor_msgs::msg::LaserScan::SharedPtr scan,
  size_t point_index,
  const geometry_msgs::msg::PoseStamped& robot_pose)
{
  geometry_msgs::msg::Point world_point;
  
  double angle = scan->angle_min + point_index * scan->angle_increment;
  double range = scan->ranges[point_index];
  
  // Convert to robot frame
  double scan_x = range * cos(angle);
  double scan_y = range * sin(angle);
  
  // Transform to world frame using robot pose
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  
  world_point.x = robot_pose.pose.position.x + scan_x * cos(robot_yaw) - scan_y * sin(robot_yaw);
  world_point.y = robot_pose.pose.position.y + scan_x * sin(robot_yaw) + scan_y * cos(robot_yaw);
  world_point.z = 0.0;
  
  return world_point;
}

void GlobalCostmapFusion::publishCostmapUpdate(const std::vector<nav2_msgs::msg::CostmapCell>& cells)
{
  auto update_msg = std::make_unique<nav2_msgs::msg::CostmapUpdate>();
  update_msg->header.stamp = this->now();
  update_msg->header.frame_id = global_frame_;
  update_msg->cells = cells;

  update_pub_->publish(std::move(update_msg));
}

void GlobalCostmapFusion::checkRobotActivity()
{
  std::lock_guard<std::mutex> lock(robot_info_mutex_);
  
  auto current_time = this->now();
  auto timeout_duration = rclcpp::Duration::from_seconds(robot_timeout_);
  
  for (auto& [robot_id, info] : robot_info_) {
    if (info.active && (current_time - info.last_update) > timeout_duration) {
      info.active = false;
      RCLCPP_WARN(this->get_logger(), "Robot %s marked as inactive due to timeout", robot_id.c_str());
    }
  }
}

}  // namespace multi_robot_costmap_plugin
