#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserManager
{
public:
    LaserManager();

    float read_front_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};
    