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
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace multi_robot_costmap_plugin
{

GlobalCostmapFusion::GlobalCostmapFusion(const rclcpp::NodeOptions & options)
: Node("global_costmap_fusion", options), map_initialized_(false)
{
  // Declare parameters
  this->declare_parameter("robots", std::vector<std::string>{"robot1"});
  this->declare_parameter("robot_radius", 0.3);
  this->declare_parameter("exclusion_buffer", 0.5);
  this->declare_parameter("update_frequency", 10.0);
  this->declare_parameter("robot_timeout", 5.0);
  this->declare_parameter("global_frame", "map");
  this->declare_parameter("output_topic", "/shared_obstacles_grid");
  this->declare_parameter("scan_topic", "scan");
  this->declare_parameter("map_topic", "/map");
  this->declare_parameter("map_service", "/map_server/map");
  this->declare_parameter("base_frame_suffix", "base_footprint");

  // Get parameters
  robot_ids_ = this->get_parameter("robots").as_string_array();
  robot_radius_ = this->get_parameter("robot_radius").as_double();
  exclusion_buffer_ = this->get_parameter("exclusion_buffer").as_double();
  update_frequency_ = this->get_parameter("update_frequency").as_double();
  robot_timeout_ = this->get_parameter("robot_timeout").as_double();
  global_frame_ = this->get_parameter("global_frame").as_string();
  scan_topic_ = this->get_parameter("scan_topic").as_string();
  map_topic_ = this->get_parameter("map_topic").as_string();
  map_service_ = this->get_parameter("map_service").as_string();
  base_frame_suffix_ = this->get_parameter("base_frame_suffix").as_string();

  // Verify robot ids are not empty
  if (robot_ids_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No robots specified in 'robots' parameter");
    throw std::runtime_error("No robots specified");
  }

  std::string output_topic = this->get_parameter("output_topic").as_string();

  // Setup TF with longer buffer for multi-robot scenarios
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
    this->get_clock(), 
    tf2::Duration(std::chrono::seconds(10))
  );
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create publisher
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    output_topic, rclcpp::QoS(10).reliable());

  // Subscribe to map topic to get map parameters
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(10).transient_local().reliable(),
    std::bind(&GlobalCostmapFusion::mapCallback, this, std::placeholders::_1));

  // Create map service client as fallback
  map_client_ = this->create_client<nav_msgs::srv::GetMap>(map_service_);

  // Initialize robot info and setup subscriptions
  setupRobotInfo();

  // Timer for checking robot activity
  activity_check_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&GlobalCostmapFusion::checkRobotActivity, this));

  // Timer for publishing shared obstacles
  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / update_frequency_)),
    std::bind(&GlobalCostmapFusion::publishSharedObstacles, this));

  // Timer for updating robot transforms
  transform_update_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), // Update transforms at 10Hz
    std::bind(&GlobalCostmapFusion::updateRobotTransforms, this));

  // Try to initialize from map
  initializeFromMap();

  RCLCPP_INFO(this->get_logger(), "GlobalCostmapFusion initialized for %zu robots", robot_ids_.size());
}

GlobalCostmapFusion::~GlobalCostmapFusion()
{
}

void GlobalCostmapFusion::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!map_initialized_) {
    // Extract map parameters
    grid_resolution_ = msg->info.resolution;
    grid_width_ = msg->info.width * grid_resolution_;
    grid_height_ = msg->info.height * grid_resolution_;
    grid_origin_ = msg->info.origin;
    global_frame_ = msg->header.frame_id;
    
    map_initialized_ = true;
    
    RCLCPP_INFO(this->get_logger(), 
                "Map parameters initialized: resolution=%.3f, width=%.2f, height=%.2f, origin=[%.2f, %.2f]",
                grid_resolution_, grid_width_, grid_height_, 
                grid_origin_.position.x, grid_origin_.position.y);
  }
}

void GlobalCostmapFusion::initializeFromMap()
{
  // Wait a bit for map topic to be available
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  if (!map_initialized_) {
    RCLCPP_INFO(this->get_logger(), "Map not received via topic, trying service...");
    requestMapFromService();
  }
}

void GlobalCostmapFusion::requestMapFromService()
{
  if (!map_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(this->get_logger(), "Map service not available");
    map_initialized_ = false;
    return;
  }

  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  
  auto future = map_client_->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    
    // Extract map parameters from service response
    const auto& map = response->map;
    grid_resolution_ = map.info.resolution;
    grid_width_ = map.info.width * grid_resolution_;
    grid_height_ = map.info.height * grid_resolution_;
    grid_origin_ = map.info.origin;
    global_frame_ = map.header.frame_id;
    
    map_initialized_ = true;
    
    RCLCPP_INFO(this->get_logger(), 
                "Map parameters from service: resolution=%.3f, width=%.2f, height=%.2f",
                grid_resolution_, grid_width_, grid_height_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get map from service");
  }
}

void GlobalCostmapFusion::setupRobotInfo()
{
  for (const auto& robot_id : robot_ids_) {
    // Create scan subscription to lidars
    std::string scan_topic = "/" + robot_id + "/" + scan_topic_;
    auto scan_callback = [this, robot_id](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      this->processLaserScan(msg, robot_id);
    };

    scan_subs_[robot_id] = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(), scan_callback);

    // Initialize robot info with TF-based approach
    RobotInfo info;
    info.transform = geometry_msgs::msg::TransformStamped();
    info.last_update = rclcpp::Time(0, 0, RCL_ROS_TIME);
    info.active = false;
    info.base_frame = robot_id + "/" + base_frame_suffix_;
    robot_info_[robot_id] = info;

    RCLCPP_INFO(this->get_logger(), "Setup subscriptions for robot: %s (base_frame: %s)", 
                robot_id.c_str(), info.base_frame.c_str());
  }
}

bool GlobalCostmapFusion::getRobotTransform(
  const std::string& robot_id, 
  geometry_msgs::msg::TransformStamped& transform)
{
  try {
    // Look up transform from robot base frame to global frame
    const auto& robot_info = robot_info_.at(robot_id);
    
    transform = tf_buffer_->lookupTransform(
      global_frame_,                  // target frame
      robot_info.base_frame,          // source frame  
      rclcpp::Time(0),               // latest available time
      rclcpp::Duration::from_seconds(0.1) // timeout
    );
    
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_DEBUG(this->get_logger(), 
                "Could not get transform for robot %s: %s", 
                robot_id.c_str(), ex.what());
    return false;
  }
}

void GlobalCostmapFusion::updateRobotTransforms()
{
  std::lock_guard<std::mutex> lock(robot_info_mutex_);
  
  for (auto& [robot_id, info] : robot_info_) {
    geometry_msgs::msg::TransformStamped transform;
    
    if (getRobotTransform(robot_id, transform)) {
      info.transform = transform;
      info.last_update = this->now();
      info.active = true;
      RCLCPP_DEBUG(this->get_logger(), "Updated transform for robot %s", robot_id.c_str());
    } else {
      // Don't immediately mark as inactive - let timeout handle it
      RCLCPP_DEBUG(this->get_logger(), "Failed to get transform for robot %s", robot_id.c_str());
    }
  }
}

void GlobalCostmapFusion::processLaserScan(
  const sensor_msgs::msg::LaserScan::SharedPtr msg,
  const std::string& robot_id)
{
  std::lock_guard<std::mutex> robot_lock(robot_info_mutex_);
  
  // Check if we have transform information for this robot
  if (robot_info_.find(robot_id) == robot_info_.end() || !robot_info_[robot_id].active) {
    RCLCPP_DEBUG(this->get_logger(), "Robot %s not active or not found", robot_id.c_str());
    return;
  }

  const auto& robot_transform = robot_info_[robot_id].transform;
  std::vector<geometry_msgs::msg::Point> new_obstacles;

  // Process each laser scan point
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max || 
        std::isnan(msg->ranges[i]) || std::isinf(msg->ranges[i])) {
      continue;
    }

    // Convert scan point to world coordinates using TF
    geometry_msgs::msg::Point world_point = convertScanPointToWorld(msg, i, robot_transform);

    // Skip if this point corresponds to another robot's location
    if (isRobotLocation(world_point)) {
      continue;
    }

    new_obstacles.push_back(world_point);
  }

  if (!new_obstacles.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "Robot %s detected %zu new obstacles", 
                robot_id.c_str(), new_obstacles.size());
  }

  // Update shared obstacles
  {
    std::lock_guard<std::mutex> obs_lock(obstacles_mutex_);
    shared_obstacles_.insert(shared_obstacles_.end(), new_obstacles.begin(), new_obstacles.end());
  }
}

bool GlobalCostmapFusion::isRobotLocation(const geometry_msgs::msg::Point& point)
{
  for (const auto& [robot_id, info] : robot_info_) {
    if (!info.active) continue;

    // Extract robot position from transform
    const auto& robot_pos = info.transform.transform.translation;
    double dx = point.x - robot_pos.x;
    double dy = point.y - robot_pos.y;
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
  const geometry_msgs::msg::TransformStamped& robot_transform)
{
  // Create a point in the laser scanner frame
  geometry_msgs::msg::PointStamped scan_point;
  scan_point.header.frame_id = scan->header.frame_id;
  scan_point.header.stamp = scan->header.stamp;
  
  double angle = scan->angle_min + point_index * scan->angle_increment;
  double range = scan->ranges[point_index];
  
  scan_point.point.x = range * cos(angle);
  scan_point.point.y = range * sin(angle);
  scan_point.point.z = 0.0;

  try {
    // Transform point from laser frame to global frame using TF2
    geometry_msgs::msg::PointStamped world_point_stamped;
    tf2::doTransform(scan_point, world_point_stamped, robot_transform);
    
    return world_point_stamped.point;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    // Return original point as fallback
    return scan_point.point;
  }
}

void GlobalCostmapFusion::publishSharedObstacles()
{
  // Don't publish until map is initialized
  if (!map_initialized_) {
    return;
  }

  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  
  if (shared_obstacles_.empty()) {
    return;
  }

  // Create occupancy grid using map-derived parameters
  auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  grid_msg->header.stamp = this->now();
  grid_msg->header.frame_id = global_frame_;

  // Use map-derived parameters
  grid_msg->info.resolution = grid_resolution_;
  grid_msg->info.width = static_cast<uint32_t>(grid_width_ / grid_resolution_);
  grid_msg->info.height = static_cast<uint32_t>(grid_height_ / grid_resolution_);
  grid_msg->info.origin = grid_origin_;

  // Initialize grid data
  grid_msg->data.resize(grid_msg->info.width * grid_msg->info.height, 0);

  // Fill in obstacles using map origin
  for (const auto& obstacle : shared_obstacles_) {
    int grid_x = static_cast<int>((obstacle.x - grid_origin_.position.x) / grid_resolution_);
    int grid_y = static_cast<int>((obstacle.y - grid_origin_.position.y) / grid_resolution_);

    if (grid_x >= 0 && grid_x < static_cast<int>(grid_msg->info.width) &&
        grid_y >= 0 && grid_y < static_cast<int>(grid_msg->info.height)) {
      int index = grid_y * grid_msg->info.width + grid_x;
      grid_msg->data[index] = 100; // Occupied
    }
  }

  grid_pub_->publish(std::move(grid_msg));
  RCLCPP_DEBUG(this->get_logger(), "Published grid with %zu obstacles", shared_obstacles_.size());

  // Clear processed obstacles
  shared_obstacles_.clear();
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

} // namespace multi_robot_costmap_plugin
