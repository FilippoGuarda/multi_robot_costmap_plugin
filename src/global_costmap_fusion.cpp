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
#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

namespace multi_robot_costmap_plugin
{

GlobalCostmapFusion::GlobalCostmapFusion(const rclcpp::NodeOptions & options)
: Node("global_costmap_fusion", options)
{
  // Declare and get basic parameters
  this->declare_parameter("robot_ids", std::vector<std::string>{"robot1", "robot2"});
  this->declare_parameter("robot_radius", 0.35);
  this->declare_parameter("exclusion_buffer", 0.1);
  this->declare_parameter("update_frequency", 10.0);
  this->declare_parameter("robot_timeout", 5.0);
  this->declare_parameter("global_frame", "map");
  this->declare_parameter("scan_topic", "scan");
  this->declare_parameter("map_topic", "map");
  this->declare_parameter("map_service", "map_server/map");
  this->declare_parameter("base_frame_suffix", "_base_link");
  
  // Memory management parameters for persistent obstacles
  this->declare_parameter("obstacle_decay_time", 30.0);
  this->declare_parameter("max_obstacle_age", 300.0);
  this->declare_parameter("min_detection_count", 2);
  this->declare_parameter("enable_memory_cleanup", true);

  // Retrieve parameters
  robot_ids_ = this->get_parameter("robot_ids").as_string_array();
  robot_radius_ = this->get_parameter("robot_radius").as_double();
  exclusion_buffer_ = this->get_parameter("exclusion_buffer").as_double();
  update_frequency_ = this->get_parameter("update_frequency").as_double();
  robot_timeout_ = this->get_parameter("robot_timeout").as_double();
  global_frame_ = this->get_parameter("global_frame").as_string();
  scan_topic_ = this->get_parameter("scan_topic").as_string();
  map_topic_ = this->get_parameter("map_topic").as_string();
  map_service_ = this->get_parameter("map_service").as_string();
  base_frame_suffix_ = this->get_parameter("base_frame_suffix").as_string();
  
  obstacle_decay_time_ = this->get_parameter("obstacle_decay_time").as_double();
  max_obstacle_age_ = this->get_parameter("max_obstacle_age").as_double();
  min_detection_count_ = this->get_parameter("min_detection_count").as_int();
  enable_memory_cleanup_ = this->get_parameter("enable_memory_cleanup").as_bool();

  // Initialize TF system
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize state
  map_initialized_ = false;

  // Create publisher for shared obstacle grid
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "shared_obstacles", rclcpp::QoS(1).reliable());

  // Create periodic timers, in newer ROS2 distrib. gotta use create_timer instead of create_wall_timer
  transform_update_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), 
    [this]() {
      try {
        this->updateRobotTransforms();
      } catch (const std::runtime_error& e) {
        RCLCPP_DEBUG(this->get_logger(), "Transform update error: %s", e.what());
      }
    });

  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / update_frequency_)),
    [this]() {
      try {
        this->publishSharedObstacles();
      } catch (const std::runtime_error& e) {
        RCLCPP_DEBUG(this->get_logger(), "Publish error: %s", e.what());
      }
    });

  activity_check_timer_ = this->create_wall_timer(
    std::chrono::seconds(3),  // 3-second delay to avoid startup race
    [this]() {
      try {
        this->checkRobotActivity();
      } catch (const std::runtime_error& e) {
        RCLCPP_DEBUG(this->get_logger(), "Activity check error: %s", e.what());
      }
    });

  if (enable_memory_cleanup_) {
    memory_cleanup_timer_ = this->create_wall_timer(
      std::chrono::seconds(15),  // 15-second delay for startup
      [this]() {
        try {
          this->cleanupOldObstacles();
        } catch (const std::runtime_error& e) {
          RCLCPP_DEBUG(this->get_logger(), "Cleanup error: %s", e.what());
        }
      });
  }

  // Setup robot info and initialize from map
  setupRobotInfo();
  initializeFromMap();

  RCLCPP_INFO(this->get_logger(), 
              "Global Costmap Fusion initialized with persistent memory for %zu robots", 
              robot_ids_.size());
}

GlobalCostmapFusion::~GlobalCostmapFusion()
{
}

void GlobalCostmapFusion::setupRobotInfo()
{
  std::lock_guard<std::mutex> lock(robot_info_mutex_);
  
  for (const auto& robot_id : robot_ids_) {
    RobotInfo info;
    info.active = false;
    info.base_frame = robot_id + base_frame_suffix_;
    info.last_update = this->get_clock()->now(); 
    robot_info_[robot_id] = info;

    // Create laser scan subscriber for each robot
    std::string full_scan_topic = "/" + robot_id + "/" + scan_topic_;
    scan_subs_[robot_id] = this->create_subscription<sensor_msgs::msg::LaserScan>(
      full_scan_topic, rclcpp::QoS(10), 
      [this, robot_id](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        this->processLaserScan(msg, robot_id);
      });
      
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s for robot %s", 
                full_scan_topic.c_str(), robot_id.c_str());
  }
}

void GlobalCostmapFusion::initializeFromMap()
{
  // Subscribe to map topic
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&GlobalCostmapFusion::mapCallback, this, std::placeholders::_1));
  
  // Also try to get map from service as fallback
  requestMapFromService();
}

void GlobalCostmapFusion::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!map_initialized_) {
    RCLCPP_INFO(this->get_logger(), "Received map, initializing persistent obstacle memory");
    
    // Store map parameters
    grid_resolution_ = msg->info.resolution;
    grid_width_ = msg->info.width * grid_resolution_;
    grid_height_ = msg->info.height * grid_resolution_;
    grid_origin_ = msg->info.origin;
    
    map_initialized_ = true;
    
    // Initialize the persistent obstacle memory
    initializeObstacleMemory();
    
    RCLCPP_INFO(this->get_logger(), 
                "Map initialized: %dx%d cells (%.2fm x %.2fm) at %.3fm resolution", 
                msg->info.width, msg->info.height, grid_width_, grid_height_, grid_resolution_);
  }
}

void GlobalCostmapFusion::requestMapFromService()
{
  map_client_ = this->create_client<nav_msgs::srv::GetMap>(map_service_);
  
  if (!map_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(this->get_logger(), "Map service %s not available, relying on topic", map_service_.c_str());
    return;
  }

  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto future = map_client_->async_send_request(request);
  
  if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
    auto response = future.get();
    auto& map = response->map;
    
    grid_resolution_ = map.info.resolution;
    grid_width_ = map.info.width * grid_resolution_;
    grid_height_ = map.info.height * grid_resolution_;
    grid_origin_ = map.info.origin;

    map_initialized_ = true;
    initializeObstacleMemory();
    
    RCLCPP_INFO(this->get_logger(), "Retrieved map from service and initialized memory");
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to get map from service, waiting for topic");
  }
}

void GlobalCostmapFusion::initializeObstacleMemory()
{
  if (!map_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Cannot initialize obstacle memory: map not ready");
    return;
  }

  if (obstacle_memory_.initialized) {
    RCLCPP_INFO(this->get_logger(), "Obstacle memory already initialized");
    return;
  }

  std::lock_guard<std::mutex> lock(obstacle_memory_.memory_mutex);

  try {
    // Calculate grid dimensions from map parameters
    int grid_width_cells = static_cast<int>(grid_width_ / grid_resolution_);
    int grid_height_cells = static_cast<int>(grid_height_ / grid_resolution_);

    // Initialize OpenCV matrices for persistent memory
    obstacle_memory_.obstacle_grid = cv::Mat::zeros(grid_height_cells, grid_width_cells, CV_8UC1);
    obstacle_memory_.timestamp_grid = cv::Mat::zeros(grid_height_cells, grid_width_cells, CV_64FC1);
    obstacle_memory_.confidence_grid = cv::Mat::zeros(grid_height_cells, grid_width_cells, CV_32FC1);

    obstacle_memory_.initialized = true;

    RCLCPP_INFO(this->get_logger(), 
                "Persistent obstacle memory initialized: %dx%d cells (%.2f x %.2f m) at %.3fm resolution",
                grid_width_cells, grid_height_cells, 
                grid_width_, grid_height_, grid_resolution_);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize obstacle memory: %s", e.what());
    obstacle_memory_.initialized = false;
  }
}

void GlobalCostmapFusion::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                                           const std::string& robot_id)
{
  if (!map_initialized_ || !obstacle_memory_.initialized) {
    return;
  }

  geometry_msgs::msg::TransformStamped robot_transform;
  if (!getRobotTransform(robot_id, robot_transform)) {
    return;
  }

  // Extract robot position
  geometry_msgs::msg::Point robot_pos;
  robot_pos.x = robot_transform.transform.translation.x;
  robot_pos.y = robot_transform.transform.translation.y;
  robot_pos.z = 0.0;

  std::vector<geometry_msgs::msg::Point> obstacles;
  std::vector<geometry_msgs::msg::Point> ray_endpoints;

  // Process each scan point
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    
    // Skip invalid readings
    if (std::isnan(range) || std::isinf(range)) {
      continue;
    }

    // Convert scan point to world coordinates
    geometry_msgs::msg::Point world_point = convertScanPointToWorld(scan, i, robot_transform);
    
    // If it's a valid obstacle detection
    if (range >= scan->range_min && range <= scan->range_max && range < scan->range_max * 0.99) {
      if (!isRobotLocation(world_point)) {
        obstacles.push_back(world_point);
      }
    }
    
    // All endpoints are used for ray tracing (whether obstacle or free space)
    ray_endpoints.push_back(world_point);
  }

  // Update persistent memory with new detections
  updateObstacleMemory(obstacles, robot_pos);
  
  // Clear obstacles along rays where we now see free space
  rayTraceClearance(robot_pos, ray_endpoints);
}

geometry_msgs::msg::Point GlobalCostmapFusion::convertScanPointToWorld(
  const sensor_msgs::msg::LaserScan::SharedPtr scan,
  size_t point_index,
  const geometry_msgs::msg::TransformStamped& robot_transform)
{
  double angle = scan->angle_min + point_index * scan->angle_increment;
  double range = scan->ranges[point_index];

  // Clamp range to maximum if needed
  if (range > scan->range_max) {
    range = scan->range_max;
  }

  // Convert to scan frame coordinates
  geometry_msgs::msg::PointStamped local_point;
  local_point.header.frame_id = scan->header.frame_id;
  local_point.point.x = range * cos(angle);
  local_point.point.y = range * sin(angle);
  local_point.point.z = 0.0;

  // Transform to world frame
  geometry_msgs::msg::PointStamped world_point;
  tf2::doTransform(local_point, world_point, robot_transform);

  return world_point.point;
}

bool GlobalCostmapFusion::getRobotTransform(const std::string& robot_id, 
                                            geometry_msgs::msg::TransformStamped& transform)
{
  try {
    transform = tf_buffer_->lookupTransform(
      global_frame_, robot_id + base_frame_suffix_, tf2::TimePointZero);
    return true;
  } catch (const tf2::TransformException& ex) {
    static std::map<std::string, int> error_count;
    error_count[robot_id]++;
    
    if (error_count[robot_id] % 10 == 1) {
      RCLCPP_WARN(this->get_logger(), "Transform failed for %s (attempt %d): %s", 
                  robot_id.c_str(), error_count[robot_id], ex.what());
    }
    return false;
  }
}

bool GlobalCostmapFusion::isRobotLocation(const geometry_msgs::msg::Point& point)
{
  std::lock_guard<std::mutex> lock(robot_info_mutex_);
  
  for (const auto& [robot_id, info] : robot_info_) {
    if (!info.active) continue;
    
    double dx = point.x - info.transform.transform.translation.x;
    double dy = point.y - info.transform.transform.translation.y;
    double distance = sqrt(dx*dx + dy*dy);
    
    if (distance < robot_radius_ + exclusion_buffer_) {
      return true;
    }
  }
  return false;
}

void GlobalCostmapFusion::updateObstacleMemory(const std::vector<geometry_msgs::msg::Point>& obstacles, 
                                               const geometry_msgs::msg::Point& robot_pos)
{
  if (!obstacle_memory_.initialized) return;
  
  std::lock_guard<std::mutex> lock(obstacle_memory_.memory_mutex);
  
  double current_time = this->get_clock()->now().seconds();
  
  // Add new obstacles to persistent memory
  for (const auto& obstacle : obstacles) {
    cv::Point2i grid_point = worldToGrid(obstacle);
    
    if (grid_point.x >= 0 && grid_point.x < obstacle_memory_.obstacle_grid.cols &&
        grid_point.y >= 0 && grid_point.y < obstacle_memory_.obstacle_grid.rows) {
      
      // Increment confidence for this detection
      float& confidence = obstacle_memory_.confidence_grid.at<float>(grid_point.y, grid_point.x);
      confidence += 1.0f;
      
      // Update last seen timestamp
      obstacle_memory_.timestamp_grid.at<double>(grid_point.y, grid_point.x) = current_time;
      
      // Mark as occupied if confidence threshold is met
      if (confidence >= min_detection_count_) {
        obstacle_memory_.obstacle_grid.at<uint8_t>(grid_point.y, grid_point.x) = 100;
      }
    }
  }
}

void GlobalCostmapFusion::rayTraceClearance(const geometry_msgs::msg::Point& robot_pos, 
                                            const std::vector<geometry_msgs::msg::Point>& endpoints)
{
  if (!obstacle_memory_.initialized) return;
  
  std::lock_guard<std::mutex> lock(obstacle_memory_.memory_mutex);
  
  cv::Point2i robot_grid = worldToGrid(robot_pos);
  
  // For each laser ray endpoint
  for (const auto& endpoint : endpoints) {
    cv::Point2i end_grid = worldToGrid(endpoint);
    
    // Get all points along the ray using Bresenham's algorithm
    std::vector<cv::Point2i> ray_points;
    rayTraceLine(robot_grid, end_grid, ray_points);
    
    // Clear obstacles along the ray (excluding the endpoint)
    for (size_t i = 0; i < ray_points.size() - 1; ++i) {
      const auto& point = ray_points[i];
      
      // Check bounds
      if (point.x >= 0 && point.x < obstacle_memory_.obstacle_grid.cols &&
          point.y >= 0 && point.y < obstacle_memory_.obstacle_grid.rows) {
        
        // Only clear cells that were previously marked as obstacles
        if (obstacle_memory_.obstacle_grid.at<uint8_t>(point.y, point.x) > 0) {
          obstacle_memory_.obstacle_grid.at<uint8_t>(point.y, point.x) = 0;
          obstacle_memory_.confidence_grid.at<float>(point.y, point.x) = 0.0f;
          // Don't reset timestamp - keep for decay tracking
        }
      }
    }
  }
}

void GlobalCostmapFusion::rayTraceLine(const cv::Point2i& start, const cv::Point2i& end, 
                                       std::vector<cv::Point2i>& line_points) const
{
  line_points.clear();
  
  // Bresenham's line algorithm for efficient ray tracing
  int x0 = start.x, y0 = start.y;
  int x1 = end.x, y1 = end.y;
  
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  
  int x = x0, y = y0;
  
  while (true) {
    line_points.emplace_back(x, y);
    
    if (x == x1 && y == y1) break;
    
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
}

cv::Point2i GlobalCostmapFusion::worldToGrid(const geometry_msgs::msg::Point& world_point) const
{
  int grid_x = static_cast<int>((world_point.x - grid_origin_.position.x) / grid_resolution_);
  int grid_y = static_cast<int>((world_point.y - grid_origin_.position.y) / grid_resolution_);
  return cv::Point2i(grid_x, grid_y);
}

geometry_msgs::msg::Point GlobalCostmapFusion::gridToWorld(const cv::Point2i& grid_point) const
{
  geometry_msgs::msg::Point world_point;
  world_point.x = grid_origin_.position.x + grid_point.x * grid_resolution_;
  world_point.y = grid_origin_.position.y + grid_point.y * grid_resolution_;
  world_point.z = 0.0;
  return world_point;
}

void GlobalCostmapFusion::updateRobotTransforms()
{
  std::lock_guard<std::mutex> lock(robot_info_mutex_);
  
  for (auto& [robot_id, info] : robot_info_) {
    geometry_msgs::msg::TransformStamped tf;
    if (getRobotTransform(robot_id, tf)) {
      info.transform = tf;
      info.last_update = this->get_clock()->now();
      info.active = true;
    }
  }
}

void GlobalCostmapFusion::checkRobotActivity()
{
  std::lock_guard<std::mutex> lock(robot_info_mutex_);
  
  if (robot_info_.empty()) {
    return;
  }
  
  try {
    int64_t current_ns = this->get_clock()->now().nanoseconds();
    int64_t timeout_ns = static_cast<int64_t>(robot_timeout_ * 1e9); // Convert seconds to nanoseconds
    
    for (auto& [robot_id, info] : robot_info_) {
      if (info.active) {
        int64_t time_diff_ns = current_ns - info.last_update.nanoseconds();
        
        if (time_diff_ns > timeout_ns) {
          info.active = false;
          RCLCPP_WARN(this->get_logger(), "Robot %s marked as inactive due to timeout", robot_id.c_str());
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Activity check failed: %s", e.what());
  }
}

void GlobalCostmapFusion::cleanupOldObstacles()
{
  if (!obstacle_memory_.initialized) return;
  
  std::lock_guard<std::mutex> lock(obstacle_memory_.memory_mutex);
  
  double current_time = this->get_clock()->now().seconds();
  int cleaned_count = 0;
  
  // Iterate through all cells and apply aging
  for (int y = 0; y < obstacle_memory_.obstacle_grid.rows; ++y) {
    for (int x = 0; x < obstacle_memory_.obstacle_grid.cols; ++x) {
      if (obstacle_memory_.obstacle_grid.at<uint8_t>(y, x) > 0) {
        double last_seen = obstacle_memory_.timestamp_grid.at<double>(y, x);
        double age = current_time - last_seen;
        
        if (age > max_obstacle_age_) {
          // Remove very old obstacles
          obstacle_memory_.obstacle_grid.at<uint8_t>(y, x) = 0;
          obstacle_memory_.confidence_grid.at<float>(y, x) = 0.0f;
          obstacle_memory_.timestamp_grid.at<double>(y, x) = 0.0;
          cleaned_count++;
        } else if (age > obstacle_decay_time_) {
          // Apply gradual decay
          float decay_factor = 1.0f - (age - obstacle_decay_time_) / (max_obstacle_age_ - obstacle_decay_time_);
          float current_confidence = obstacle_memory_.confidence_grid.at<float>(y, x);
          float new_confidence = current_confidence * decay_factor;
          
          if (new_confidence < min_detection_count_) {
            obstacle_memory_.obstacle_grid.at<uint8_t>(y, x) = 0;
            obstacle_memory_.confidence_grid.at<float>(y, x) = 0.0f;
            cleaned_count++;
          } else {
            obstacle_memory_.confidence_grid.at<float>(y, x) = new_confidence;
          }
        }
      }
    }
  }
  
  if (cleaned_count > 0) {
    RCLCPP_DEBUG(this->get_logger(), "Cleaned up %d old obstacles from memory", cleaned_count);
  }
}

void GlobalCostmapFusion::publishSharedObstacles()
{
  if (!map_initialized_ || !obstacle_memory_.initialized) {
    return;
  }
  
  std::lock_guard<std::mutex> lock(obstacle_memory_.memory_mutex);
  
  // Create occupancy grid message
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = this->get_clock()->now();
  grid.header.frame_id = global_frame_;
  
  grid.info.resolution = grid_resolution_;
  grid.info.width = obstacle_memory_.obstacle_grid.cols;
  grid.info.height = obstacle_memory_.obstacle_grid.rows;
  grid.info.origin = grid_origin_;
  
  // Convert OpenCV matrix to occupancy grid data
  grid.data.resize(grid.info.width * grid.info.height);
  for (int y = 0; y < obstacle_memory_.obstacle_grid.rows; ++y) {
    for (int x = 0; x < obstacle_memory_.obstacle_grid.cols; ++x) {
      int index = y * grid.info.width + x;
      grid.data[index] = obstacle_memory_.obstacle_grid.at<uint8_t>(y, x);
    }
  }
  
  // Mark robot footprints as free space
  {
    std::lock_guard<std::mutex> robot_lock(robot_info_mutex_);
    for (const auto& [robot_id, info] : robot_info_) {
      if (info.active) {
        markRobotFootprint(grid, 
                          info.transform.transform.translation.x,
                          info.transform.transform.translation.y);
      }
    }
  }
  
  grid_pub_->publish(grid);
}

void GlobalCostmapFusion::markRobotFootprint(nav_msgs::msg::OccupancyGrid& grid, 
                                             double robot_x, double robot_y)
{
  geometry_msgs::msg::Point robot_point;
  robot_point.x = robot_x;
  robot_point.y = robot_y;
  
  cv::Point2i robot_grid = worldToGrid(robot_point);
  int radius_cells = static_cast<int>((robot_radius_ + exclusion_buffer_) / grid_resolution_) + 1;
  
  // Clear circular area around robot
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      int grid_x = robot_grid.x + dx;
      int grid_y = robot_grid.y + dy;
      
      if (grid_x >= 0 && grid_x < static_cast<int>(grid.info.width) &&
          grid_y >= 0 && grid_y < static_cast<int>(grid.info.height)) {
        
        double distance = sqrt(dx*dx + dy*dy) * grid_resolution_;
        if (distance <= robot_radius_ + exclusion_buffer_) {
          int index = grid_y * grid.info.width + grid_x;
          grid.data[index] = 0;  // Mark as free
        }
      }
    }
  }
}

} // namespace multi_robot_costmap_plugin
