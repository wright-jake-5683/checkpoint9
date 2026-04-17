#pragma once

#include "laser_readings.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <vector>

class LaserManager
{
public:
    LaserManager();

    float read_front_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    float find_angle_from_laser_reading(sensor_msgs::msg::LaserScan::SharedPtr msg, int index);

    std::vector<std::vector<LaserReadings>> cluster_laser_data(const std::vector<float> &readings);
};