#include <chrono>
#include <memory>
#include <cmath>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters

  //first attempt at lidar subs

  lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    10,
    std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

    costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  

  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}


 void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "data recieved");
  costmap_.processLaserScan(msg);
  auto occupancy_grid = costmap_.getOccupancyGrid();
  costmap_publisher_ -> publish(occupancy_grid);

  //calm debug
  RCLCPP_INFO(this->get_logger(), "Published updated costmap");
 }


// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "RUNNINGSSS";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
