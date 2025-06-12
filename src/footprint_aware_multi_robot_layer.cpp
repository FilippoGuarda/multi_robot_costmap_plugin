#include "multi_robot_costmap_plugin/footprint_aware_multi_robot_layer.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace multi_robot_costmap_plugin
{

FootprintAwareMultiRobotLayer::FootprintAwareMultiRobotLayer()
: nav2_costmap_2d::CostmapLayer()
{
  // Constructor implementation
}

void FootprintAwareMultiRobotLayer::onInitialize()
{
  auto node = node_.lock();
  
  // Declare parameters
  declareParameter("robot_namespaces", rclcpp::ParameterValue(std::vector<std::string>{}));
  declareParameter("obstacle_max_range", rclcpp::ParameterValue(2.5));
  declareParameter("obstacle_min_range", rclcpp::ParameterValue(0.0));
  declareParameter("global_frame", rclcpp::ParameterValue("map"));
  declareParameter("footprint_padding", rclcpp::ParameterValue(0.1));
  
  // Get parameters
  node->get_parameter(name_ + ".robot_namespaces", robot_namespaces_);
  node->get_parameter(name_ + ".obstacle_max_range", obstacle_max_range_);
  node->get_parameter(name_ + ".obstacle_min_range", obstacle_min_range_);
  node->get_parameter(name_ + ".global_frame", global_frame_);
  node->get_parameter(name_ + ".footprint_padding", footprint_padding_);
  
  // Subscribe to sensor data from all robots
  for (const auto& robot_ns : robot_namespaces_) {
    // Laser scan subscription
    auto scan_callback = [this, robot_ns](const sensor_msgs::msg::LaserScan::SharedPtr scan) {
      this->laserScanCallback(scan, robot_ns);
    };
    std::string scan_topic = "/" + robot_ns + "/scan";
    laser_subs_.push_back(
      node->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, scan_callback)
    );
    
    // Robot pose subscription
    auto pose_callback = [this, robot_ns](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose) {
      this->robotPoseCallback(pose, robot_ns);
    };
    std::string pose_topic = "/" + robot_ns + "/amcl_pose";
    pose_subs_.push_back(
      node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 10, pose_callback)
    );
  }
  
  current_ = true;
  enabled_ = true;
}

void FootprintAwareMultiRobotLayer::activate()
{
  enabled_ = true;
  // Any additional activation logic
}

void FootprintAwareMultiRobotLayer::deactivate()
{
  enabled_ = false;
  // Any additional deactivation logic
}

void FootprintAwareMultiRobotLayer::onFootprintChanged()
{
  // Handle footprint changes if needed
  RCLCPP_DEBUG(logger_, "Robot footprint changed");
}

void FootprintAwareMultiRobotLayer::matchSize()
{
  // This method is called when the master costmap size changes
  // Typically, you don't need to do anything special here for observation layers
  nav2_costmap_2d::CostmapLayer::matchSize();
}

void FootprintAwareMultiRobotLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                                double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_) return;
  
  // Update bounds based on laser scan data
  for (const auto& [robot_id, scan] : latest_scans_) {
    if (!scan) continue;
    
    // Expand bounds to include scan range
    *min_x = std::min(*min_x, robot_x - obstacle_max_range_);
    *min_y = std::min(*min_y, robot_y - obstacle_max_range_);
    *max_x = std::max(*max_x, robot_x + obstacle_max_range_);
    *max_y = std::max(*max_y, robot_y + obstacle_max_range_);
  }
}

void FootprintAwareMultiRobotLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                                               int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;
  
  for (const auto& [robot_id, scan] : latest_scans_) {
    if (!scan) continue;
    
    // Convert laser scan to point cloud
    sensor_msgs::msg::PointCloud2 cloud;
    try {
      projector_.projectLaser(*scan, cloud);
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Failed to project laser scan from %s: %s", 
                  robot_id.c_str(), e.what());
      continue;
    }
    
    // Process each point in the cloud
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");

    
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      double point_x = *iter_x;
      double point_y = *iter_y;
      
      // Skip points that are inside any robot's footprint
      if (isPointInsideRobotFootprint(point_x, point_y, robot_id)) {
        continue;
      }
      
      // Mark obstacle in costmap
      unsigned int mx, my;
      if (master_grid.worldToMap(point_x, point_y, mx, my)) {
        if (static_cast<int>(mx) >= min_i && static_cast<int>(mx) <= max_i && 
        static_cast<int>(my) >= min_j && static_cast<int>(my) <= max_j){
          master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
}

void FootprintAwareMultiRobotLayer::reset()
{
  // Clear stored scan data
  latest_scans_.clear();
  // Note: Don't clear robot_poses_ as they're needed for footprint filtering
}

// Callback implementations
void FootprintAwareMultiRobotLayer::laserScanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr scan, 
  const std::string& robot_id)
{
  latest_scans_[robot_id] = scan;
}

void FootprintAwareMultiRobotLayer::robotPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose,
  const std::string& robot_id)
{
  robot_poses_[robot_id] = pose;
}

// Utility method implementations
bool FootprintAwareMultiRobotLayer::isPointInsideRobotFootprint(double x, double y, 
                                                               const std::string& exclude_robot_id)
{
  for (const auto& [robot_id, pose_msg] : robot_poses_) {
    if (robot_id == exclude_robot_id || !pose_msg) continue;
    
    // Create simple circular footprint for now (can be enhanced)
    double robot_radius = 0.3 + footprint_padding_;
    double dx = x - pose_msg->pose.pose.position.x;
    double dy = y - pose_msg->pose.pose.position.y;
    double distance = sqrt(dx*dx + dy*dy);
    
    if (distance < robot_radius) {
      return true;
    }
  }
  return false;
}

bool FootprintAwareMultiRobotLayer::isPointInsidePolygon(double x, double y, 
                                                        const std::vector<geometry_msgs::msg::Point32>& polygon)
{
  // Ray casting algorithm implementation
  int num_vertices = polygon.size();
  if (num_vertices < 3) return false;
  
  bool inside = false;
  int j = num_vertices - 1;
  
  for (int i = 0; i < num_vertices; i++) {
    double xi = polygon[i].x, yi = polygon[i].y;
    double xj = polygon[j].x, yj = polygon[j].y;
    
    if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
    j = i;
  }
  
  return inside;
}

geometry_msgs::msg::Polygon FootprintAwareMultiRobotLayer::addFootprintPadding(
    const geometry_msgs::msg::Polygon& footprint, double padding)
{
  geometry_msgs::msg::Polygon padded = footprint;
  // Simple uniform padding logic (expand points radially)
  for (auto& pt : padded.points) {
    double mag = std::hypot(pt.x, pt.y);
    if (mag > 1e-6) {
      pt.x *= (mag + padding) / mag;
      pt.y *= (mag + padding) / mag;
    }
  }
  return padded;
}

geometry_msgs::msg::Polygon FootprintAwareMultiRobotLayer::transformFootprintToGlobal(
  const geometry_msgs::msg::Polygon& footprint, 
  const geometry_msgs::msg::Pose& robot_pose)
{
  geometry_msgs::msg::Polygon global_footprint;
  
  // Extract yaw from quaternion
  const auto& q = robot_pose.orientation;
  double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  
  double cos_theta = cos(yaw);
  double sin_theta = sin(yaw);
  
  for (const auto& point : footprint.points) {
    geometry_msgs::msg::Point32 global_point;
    
    global_point.x = robot_pose.position.x + (point.x * cos_theta - point.y * sin_theta);
    global_point.y = robot_pose.position.y + (point.x * sin_theta + point.y * cos_theta);
    global_point.z = 0.0;
    
    global_footprint.points.push_back(global_point);
  }
  
  return global_footprint;
}

}  // namespace multi_robot_costmap_plugin

// Plugin registration
PLUGINLIB_EXPORT_CLASS(multi_robot_costmap_plugin::FootprintAwareMultiRobotLayer, nav2_costmap_2d::Layer)
