#include "laser_manager.hpp"

LaserManager::LaserManager()
{}

float LaserManager::read_front_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (msg->ranges.empty()) {
          throw std::runtime_error("No laser data received yet");
    }

    int middle_index = std::round(msg->ranges.size() / 2);
    return msg->ranges[middle_index];
}

float LaserManager::find_angle_from_laser_reading(sensor_msgs::msg::LaserScan::SharedPtr msg, int index)
{
    float angle = msg->angle_min + (index * msg->angle_increment);
    return angle;
}
