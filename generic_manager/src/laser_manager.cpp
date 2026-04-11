#include "laser_manager.hpp"

LaserManager::LaserManager() {}

float LaserManager::read_front_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int middle_index = std::round(std::size(msg->ranges) / 2);
    return msg->ranges[middle_index];
}

