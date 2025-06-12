#ifndef MULTI_ROBOT_COSTMAP_PLUGIN__FOOTPRINT_AWARE_MULTI_ROBOT_LAYER_HPP_
#define MULTI_ROBOT_COSTMAP_PLUGIN__FOOTPRINT_AWARE_MULTI_ROBOT_LAYER_HPP_

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace multi_robot_costmap_plugin
{
class FootprintAwareMultiRobotLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  FootprintAwareMultiRobotLayer();
  virtual ~FootprintAwareMultiRobotLayer() = default;
  
  // Core Layer interface methods - REQUIRED
  virtual void onInitialize() override;
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, 
                           double* min_x, double* min_y, double* max_x, double* max_y) override;
  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, 
                          int min_i, int min_j, int max_i, int max_j) override;
  virtual void reset() override;
  
  // Lifecycle methods - REQUIRED for CostmapLayer
  virtual void activate() override;
  virtual void deactivate() override;
  virtual void onFootprintChanged() override;
  
  // Additional methods that may be required
  virtual void matchSize() override;
  virtual bool isClearable() override { return true; }

  // Test checks for footprint based point removal
  virtual bool isPointInsidePolygon(double x, double y, const std::vector<geometry_msgs::msg::Point32>& polygon);
  geometry_msgs::msg::Polygon transformFootprintToGlobal(
      const geometry_msgs::msg::Polygon& footprint, 
      const geometry_msgs::msg::Pose& robot_pose);
  geometry_msgs::msg::Polygon addFootprintPadding(const geometry_msgs::msg::Polygon& footprint, double padding);
  
  
private:
  // Sensor data callbacks
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, 
                        const std::string& robot_id);
  void robotPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose,
                        const std::string& robot_id);
  
  // Footprint checking utilities
  bool isPointInsideRobotFootprint(double x, double y, const std::string& exclude_robot_id);
  
  
  // Data storage
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> laser_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> pose_subs_;
  
  std::map<std::string, sensor_msgs::msg::LaserScan::SharedPtr> latest_scans_;
  std::map<std::string, geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> robot_poses_;
  std::map<std::string, geometry_msgs::msg::Polygon> robot_footprints_;
  
  std::vector<std::string> robot_namespaces_;
  laser_geometry::LaserProjection projector_;
  
  std::string global_frame_;
  double obstacle_max_range_;
  double obstacle_min_range_;
  double footprint_padding_;
};
}

#endif   // MULTI_ROBOT_COSTMAP_PLUGIN__FOOTPRINT_AWARE_MULTI_ROBOT_LAYER_HPP_