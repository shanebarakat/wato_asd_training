#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector> 
#include <sensor_msgs/msg/laser_scan.hpp> 
#include <nav_msgs/msg/occupancy_grid.hpp> 
#include "rclcpp/rclcpp.hpp" 

namespace robot {

class CostmapCore {
  public:
    
    explicit CostmapCore(const rclcpp::Logger& logger);
    void initializeCostmap(int width, int height, double resolution);
    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan);
    nav_msgs::msg::OccupancyGrid getOccupancyGrid();

  private:
    
    rclcpp::Logger logger_;
    int width_; 
    int height_; 
    double resolution_; 
    std::vector<int8_t> costmap_;

    void markObstacle(int x, int y);

    void inflateObstacles();

    void convertToGrid(double range, double angle, int &x, int &y);
};

}

#endif 