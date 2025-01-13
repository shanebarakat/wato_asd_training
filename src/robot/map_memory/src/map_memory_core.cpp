#include "map_memory_core.hpp"

namespace robot {


MapMemoryCore::MapMemoryCore() {}


void MapMemoryCore::initializeGlobalMap(int width, int height, double resolution) {
    global_map_.info.resolution = resolution; 
    global_map_.info.width = width;        
    global_map_.info.height = height;       
    global_map_.data.resize(width * height, -1);
}


void MapMemoryCore::mergeCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                 double robot_x, double robot_y) {
    for (size_t y = 0; y < costmap.info.height; ++y) {
        for (size_t x = 0; x < costmap.info.width; ++x) {
            
            int global_x, global_y;
            transformToGlobal(costmap, x, y, robot_x, robot_y, global_x, global_y);

            
            if (global_x >= 0 && global_x < global_map_.info.width &&
                global_y >= 0 && global_y < global_map_.info.height) {
                size_t global_index = global_y * global_map_.info.width + global_x;
                size_t costmap_index = y * costmap.info.width + x;

                
                if (costmap.data[costmap_index] != -1) { 
                    global_map_.data[global_index] = costmap.data[costmap_index];
                }
            }
        }
    }
}


void MapMemoryCore::transformToGlobal(const nav_msgs::msg::OccupancyGrid& costmap, int local_x,
                                      int local_y, double robot_x, double robot_y,
                                      int& global_x, int& global_y) {
    
    double local_x_world = local_x * costmap.info.resolution + costmap.info.origin.position.x;
    double local_y_world = local_y * costmap.info.resolution + costmap.info.origin.position.y;

   
    double global_x_world = local_x_world + robot_x;
    double global_y_world = local_y_world + robot_y;

    
    global_x = static_cast<int>(global_x_world / global_map_.info.resolution);
    global_y = static_cast<int>(global_y_world / global_map_.info.resolution);
}


const nav_msgs::msg::OccupancyGrid& MapMemoryCore::getGlobalMap() const {
    return global_map_;
}

}
