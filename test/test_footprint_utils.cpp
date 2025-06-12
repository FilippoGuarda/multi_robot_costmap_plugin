#include <gtest/gtest.h>
#include <memory>
#include "multi_robot_costmap_plugin/footprint_aware_multi_robot_layer.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"

class FootprintUtilsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_node");
    layer_ = std::make_shared<multi_robot_costmap_plugin::FootprintAwareMultiRobotLayer>();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<multi_robot_costmap_plugin::FootprintAwareMultiRobotLayer> layer_;
};

TEST_F(FootprintUtilsTest, TestPointInsideSquarePolygon)
{
  // Create a square polygon
  std::vector<geometry_msgs::msg::Point32> square;
  geometry_msgs::msg::Point32 p1, p2, p3, p4;
  
  p1.x = -1.0; p1.y = -1.0; p1.z = 0.0;
  p2.x = 1.0;  p2.y = -1.0; p2.z = 0.0;
  p3.x = 1.0;  p3.y = 1.0;  p3.z = 0.0;
  p4.x = -1.0; p4.y = 1.0;  p4.z = 0.0;
  
  square.push_back(p1);
  square.push_back(p2);
  square.push_back(p3);
  square.push_back(p4);

  // Test points inside the square
  EXPECT_TRUE(layer_->isPointInsidePolygon(0.0, 0.0, square));
  EXPECT_TRUE(layer_->isPointInsidePolygon(0.5, 0.5, square));
  EXPECT_TRUE(layer_->isPointInsidePolygon(-0.5, -0.5, square));

  // Test points outside the square
  EXPECT_FALSE(layer_->isPointInsidePolygon(2.0, 0.0, square));
  EXPECT_FALSE(layer_->isPointInsidePolygon(0.0, 2.0, square));
  EXPECT_FALSE(layer_->isPointInsidePolygon(-2.0, -2.0, square));

  // Test points on the edge (implementation dependent)
  EXPECT_FALSE(layer_->isPointInsidePolygon(1.0, 0.0, square));
}

TEST_F(FootprintUtilsTest, TestCircularFootprintCreation)
{
  geometry_msgs::msg::Polygon footprint;
  double radius = 0.5;
  
  layer_->createCircularFootprint(footprint, radius);
  
  // Check that we have the expected number of points
  EXPECT_EQ(footprint.points.size(), 12);
  
  // Check that all points are approximately at the correct distance
  for (const auto& point : footprint.points) {
    double distance = sqrt(point.x * point.x + point.y * point.y);
    EXPECT_NEAR(distance, radius, 1e-6);
  }
}

TEST_F(FootprintUtilsTest, TestFootprintTransformation)
{
  // Create a simple square footprint
  geometry_msgs::msg::Polygon footprint;
  geometry_msgs::msg::Point32 p1, p2, p3, p4;
  
  p1.x = -0.5; p1.y = -0.5; p1.z = 0.0;
  p2.x = 0.5;  p2.y = -0.5; p2.z = 0.0;
  p3.x = 0.5;  p3.y = 0.5;  p3.z = 0.0;
  p4.x = -0.5; p4.y = 0.5;  p4.z = 0.0;
  
  footprint.points.push_back(p1);
  footprint.points.push_back(p2);
  footprint.points.push_back(p3);
  footprint.points.push_back(p4);

  // Create a robot pose
  geometry_msgs::msg::Pose robot_pose;
  robot_pose.position.x = 1.0;
  robot_pose.position.y = 2.0;
  robot_pose.position.z = 0.0;
  robot_pose.orientation.x = 0.0;
  robot_pose.orientation.y = 0.0;
  robot_pose.orientation.z = 0.0;
  robot_pose.orientation.w = 1.0; // No rotation

  // Transform footprint
  geometry_msgs::msg::Polygon transformed = layer_->transformFootprintToGlobal(footprint, robot_pose);
  
  // Check that points are correctly translated
  EXPECT_EQ(transformed.points.size(), 4);
  EXPECT_NEAR(transformed.points[0].x, 0.5, 1e-6);  // -0.5 + 1.0
  EXPECT_NEAR(transformed.points[0].y, 1.5, 1e-6);  // -0.5 + 2.0
}

TEST_F(FootprintUtilsTest, TestFootprintPadding)
{
  // Create a simple triangle
  geometry_msgs::msg::Polygon footprint;
  geometry_msgs::msg::Point32 p1, p2, p3;
  
  p1.x = 0.0;  p1.y = 1.0;  p1.z = 0.0;
  p2.x = -1.0; p2.y = -1.0; p2.z = 0.0;
  p3.x = 1.0;  p3.y = -1.0; p3.z = 0.0;
  
  footprint.points.push_back(p1);
  footprint.points.push_back(p2);
  footprint.points.push_back(p3);

  double padding = 0.1;
  geometry_msgs::msg::Polygon padded = layer_->addFootprintPadding(footprint, padding);
  
  // Check that we still have the same number of points
  EXPECT_EQ(padded.points.size(), 3);
  
  // Calculate original centroid
  double cx = (p1.x + p2.x + p3.x) / 3.0;
  double cy = (p1.y + p2.y + p3.y) / 3.0;
  
  // Check that padded points are further from centroid
  for (size_t i = 0; i < footprint.points.size(); ++i) {
    double orig_dist = sqrt(pow(footprint.points[i].x - cx, 2) + pow(footprint.points[i].y - cy, 2));
    double padded_dist = sqrt(pow(padded.points[i].x - cx, 2) + pow(padded.points[i].y - cy, 2));
    
    EXPECT_GT(padded_dist, orig_dist);
    EXPECT_NEAR(padded_dist - orig_dist, padding, 1e-6);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}