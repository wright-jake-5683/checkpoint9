#pragma once

#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

class LaserManager
{
public:
    LaserManager();

    float read_front_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    float find_angle_from_laser(sensor_msgs::msg::LaserScan::SharedPtr msg, int index)
};