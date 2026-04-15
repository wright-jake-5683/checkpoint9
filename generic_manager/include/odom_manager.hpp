#pragma once
#include <cmath>

#include <nav_msgs/msg/odometry.hpp> 
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "rpy.hpp"

class OdomManager
{
public:
    OdomManager();

    RPY get_rpy(const nav_msgs::msg::Odometry::SharedPtr msg);

    float convert_degrees_to_radians(float degrees);

    float convert_radians_to_degrees(float radians);

    float normalize_angle(float angle);
};