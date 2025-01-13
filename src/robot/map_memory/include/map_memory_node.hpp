#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp" 
#include "nav_msgs/msg/odometry.hpp" 
#include <cmath> 


class MapMemoryNode : public rclcpp::Node {
public:
 
    MapMemoryNode();

private:
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

   
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::OccupancyGrid global_map_; 
    nav_msgs::msg::OccupancyGrid latest_costmap_; 
    double last_x_, last_y_; 
    const double distance_threshold_; 
    bool costmap_updated_; 
    bool should_update_map_; 
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();
    void integrateCostmap();
};

#endif 
