#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>

namespace robot {

class MapMemoryCore {
public:
    
    explicit MapMemoryCore();

    
    void initializeGlobalMap(int width, int height, double resolution);

    
    void mergeCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y);

   
    const nav_msgs::msg::OccupancyGrid& getGlobalMap() const;

private:
    
    nav_msgs::msg::OccupancyGrid global_map_;

    
    void transformToGlobal(const nav_msgs::msg::OccupancyGrid& costmap, int local_x, int local_y,
                           double robot_x, double robot_y, int& global_x, int& global_y);
};

} // namespace robot

#endif // MAP_MEMORY_CORE_HPP_

