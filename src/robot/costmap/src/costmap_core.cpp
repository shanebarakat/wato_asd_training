#include "costmap_core.hpp"
#include <cmath> costmap

namespace robot {


CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initializeCostmap(int width, int height, double resolution) {
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    costmap_.resize(width_ * height_, 0); 
    RCLCPP_INFO(logger_, "Initialized costmap with dimensions %dx%d and resolution %.2f meters per cell.", width_, height_, resolution_);
}


void CostmapCore::convertToGrid(double range, double angle, int &x, int &y) {

    double x_world = range * std::cos(angle);
    double y_world = range * std::sin(angle);

  
    x = static_cast<int>((x_world + (width_ * resolution_) / 2) / resolution_);
    y = static_cast<int>((y_world + (height_ * resolution_) / 2) / resolution_);
}


void CostmapCore::markObstacle(int x, int y) {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        costmap_[y * width_ + x] = 100; 
    }
}

void CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan) {
    RCLCPP_INFO(logger_, "Processing LaserScan data");

    std::fill(costmap_.begin(), costmap_.end(), 0);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (range > scan->range_min && range < scan->range_max) {
            int x, y;
            convertToGrid(range, angle, x, y);
            markObstacle(x, y);
        }
    }

  
    inflateObstacles();
}

void CostmapCore::inflateObstacles() {
    std::vector<int8_t> inflated_costmap = costmap_;
    int inflation_radius = static_cast<int>(1.0 / resolution_);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (costmap_[y * width_ + x] == 100) { 
                for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                    for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                            if (distance <= 1.0) { 
                                int cost = static_cast<int>(100 * (1.0 - distance / 1.0));
                                inflated_costmap[ny * width_ + nx] = std::max(
                                    inflated_costmap[ny * width_ + nx], cost);
                            }
                        }
                    }
                }
            }
        }
    }

    costmap_ = inflated_costmap;
}


nav_msgs::msg::OccupancyGrid CostmapCore::getOccupancyGrid() {
    nav_msgs::msg::OccupancyGrid grid;

    
    grid.header.frame_id = "map";
    grid.info.resolution = resolution_;
    grid.info.width = width_;
    grid.info.height = height_;
    grid.info.origin.position.x = -(width_ * resolution_) / 2;
    grid.info.origin.position.y = -(height_ * resolution_) / 2;
    grid.data = costmap_;

    return grid;
}

} // namespace robot
