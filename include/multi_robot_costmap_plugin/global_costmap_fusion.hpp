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


#ifndef MULTI_ROBOT_COSTMAP_PLUGIN__GLOBAL_COSTMAP_FUSION_HPP_
#define MULTI_ROBOT_COSTMAP_PLUGIN__GLOBAL_COSTMAP_FUSION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/msg/costmap_update.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <map>
#include <vector>
#include <string>
#include <mutex>

namespace multi_robot_costmap_plugin
{

class GlobalCostmapFusion : public rclcpp::Node
{
public:
  explicit GlobalCostmapFusion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~GlobalCostmapFusion();

private:
  struct RobotInfo {
    geometry_msgs::msg::PoseStamped pose;
    rclcpp::Time last_update;
    bool active;
  };

  void setupSubscriptions();
  void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string& robot_id);
  void updateRobotPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string& robot_id);
  bool isRobotLocation(const geometry_msgs::msg::Point& point);
  geometry_msgs::msg::Point convertScanPointToWorld(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    size_t point_index,
    const geometry_msgs::msg::PoseStamped& robot_pose);
  void publishCostmapUpdate(const std::vector<nav2_msgs::msg::CostmapCell>& cells);
  void checkRobotActivity();

  // Parameters
  std::vector<std::string> robot_ids_;
  double robot_radius_;
  double exclusion_buffer_;
  double update_frequency_;
  double robot_timeout_;
  std::string global_frame_;

  // Publishers and subscribers
  rclcpp::Publisher<nav2_msgs::msg::CostmapUpdate>::SharedPtr update_pub_;
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subs_;
  std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;

  // Robot tracking
  std::map<std::string, RobotInfo> robot_info_;
  std::mutex robot_info_mutex_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timers
  rclcpp::TimerBase::SharedPtr activity_check_timer_;
};

}  // namespace multi_robot_costmap_plugin

#endif  // MULTI_ROBOT_COSTMAP_PLUGIN__GLOBAL_COSTMAP_FUSION_HPP_
