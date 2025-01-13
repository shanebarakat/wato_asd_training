#include "map_memory_node.hpp"


MapMemoryNode::MapMemoryNode()
    : Node("map_memory_node"),
      last_x_(0.0),
      last_y_(0.0),
      distance_threshold_(1.5),
      costmap_updated_(false),
      should_update_map_(false) {
    
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

    
    global_map_.info.resolution = 0.1; 
    global_map_.info.width = 100;     
    global_map_.info.height = 100;  
    global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1); 
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg; 
    costmap_updated_ = true; 
    RCLCPP_INFO(this->get_logger(), "Received new costmap");
}


void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x; 
    double y = msg->pose.pose.position.y;

    
    double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
    if (distance >= distance_threshold_) { 
        last_x_ = x;
        last_y_ = y;
        should_update_map_ = true; 
        RCLCPP_INFO(this->get_logger(), "Robot moved %.2f meters, triggering map update.", distance);
    }
}


void MapMemoryNode::updateMap() {
    if (should_update_map_ && costmap_updated_) {
        integrateCostmap(); 
        map_pub_->publish(global_map_);
        should_update_map_ = false; 
        RCLCPP_INFO(this->get_logger(), "Global map updated and published");
    }
}


void MapMemoryNode::integrateCostmap() {
    for (size_t y = 0; y < latest_costmap_.info.height; ++y) {
        for (size_t x = 0; x < latest_costmap_.info.width; ++x) {
        
            int global_x = x + latest_costmap_.info.origin.position.x / global_map_.info.resolution;
            int global_y = y + latest_costmap_.info.origin.position.y / global_map_.info.resolution;

            if (global_x >= 0 && global_x < global_map_.info.width &&
                global_y >= 0 && global_y < global_map_.info.height) {
                size_t global_index = global_y * global_map_.info.width + global_x;
                size_t costmap_index = y * latest_costmap_.info.width + x;

                if (latest_costmap_.data[costmap_index] != -1) {
                    global_map_.data[global_index] = latest_costmap_.data[costmap_index];
                }
            }
        }
    }
}
